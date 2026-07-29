#!/usr/bin/env python3
"""
waypoint_recorder.py — 循迹路点录制工具
========================================
用法:
  1. 确认 /ins/nav 已发布 (pbox_node + ins_nav_node 运行中, RTK 固定解最佳)
  2. ros2 run vb300_nodes waypoint_recorder
  3. 遥控车沿路线行驶, 自动按间距打点 (或手动补点)
  4. 按 S 保存, Q 保存并退出

按键:
  空格/回车  手动打一个点 (弯道处补点)
  A          自动打点开关 (默认开, 每 min_dist 米一点)
  O          把当前位置重设为原点 (谨慎: 已打的点会按新原点重算)
  S          保存到文件
  Q          保存并退出

输出:
  waypoints.yaml (x/y/speed 与 planner 格式一致), 并在头部注释给出
  origin_lat/origin_lon —— 必须抄到 vehicle.yaml 的 ins_nav_node 节,
  之后每次启动用同一原点, 路点不会漂移。
"""

import sys
import math
import select
import termios
import tty
import time
import yaml
import rclpy
from rclpy.node import Node
from vb300_interfaces.msg import InsNav


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('min_dist', 1.0)        # 自动打点间距 [m]
        self.declare_parameter('default_speed', 0.5)   # 路点默认速度 [m/s]
        self.declare_parameter('utm_zone', 50)
        self.declare_parameter('northern', True)
        self.declare_parameter('output',
            '/home/cabr/vb300_ws/src/vb300_nodes/config/waypoints_recorded.yaml')

        self.create_subscription(InsNav, '/ins/nav', self._on_nav, 10)
        self.cur = None              # 最新 InsNav
        self.origin = None           # (lat, lon, alt)
        self.points = []             # [(x, y, speed)]
        self.auto = True
        self.running = True
        self._last_xy = None

        self.get_logger().info('路点录制已启动, 等待 /ins/nav ...')

    # ---------------- 数据 ----------------

    def _on_nav(self, msg: InsNav):
        self.cur = msg

    # ---------------- 主循环 ----------------

    def run(self):
        fd = sys.stdin.fileno()
        interactive = fd >= 0 and self._is_tty(fd)
        old = None
        if interactive:
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        else:
            self.get_logger().warn('stdin 非终端, 仅自动打点, Ctrl+C 保存退出')
        last_warn = 0
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                m = self.cur
                if m is None:
                    continue
                # 状态提示
                if not m.aligned or m.position_type not in (48, 49, 50):
                    if time.time() - last_warn > 5:
                        last_warn = time.time()
                        self.get_logger().warn(
                            f'非RTK固定解 (status={m.status} type={m.position_type}), '
                            f'建议等固定解再录')
                # 原点未设: 用第一个有效点
                if self.origin is None and m.aligned:
                    self.origin = (m.latitude, m.longitude, m.altitude)
                    self.get_logger().info(
                        f'原点: {m.latitude:.7f}, {m.longitude:.7f}')
                if self.origin is None:
                    continue
                x, y = self._project(m.latitude, m.longitude)
                self._last_xy = (x, y)
                # 自动打点
                if self.auto and (not self.points or
                        math.hypot(x - self.points[-1][0],
                                   y - self.points[-1][1]) >= self.get_parameter('min_dist').value):
                    self._add_point(x, y, auto=True)
                # 按键
                if interactive and select.select([fd], [], [], 0)[0]:
                    self._on_key(fd)
        finally:
            if old is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

    @staticmethod
    def _is_tty(fd):
        try:
            termios.tcgetattr(fd)
            return True
        except termios.error:
            return False

    def _on_key(self, fd):
        key = sys.stdin.read(1)
        if key in (' ', '\n') and self._last_xy:
            self._add_point(*self._last_xy, auto=False)
        elif key in ('a', 'A'):
            self.auto = not self.auto
            print(f'自动打点: {"开" if self.auto else "关"}')
        elif key in ('o', 'O') and self.cur and self.cur.aligned:
            self.origin = (self.cur.latitude, self.cur.longitude, self.cur.altitude)
            lat0, lon0, _ = self.origin
            # 已打点按新原点重算没有意义, 清空重录
            self.points.clear()
            print(f'原点重设为 {lat0:.7f}, {lon0:.7f}, 已清空旧点')
        elif key in ('s', 'S'):
            self.save()
        elif key in ('q', 'Q'):
            self.save()
            self.running = False

    # ---------------- 打点/保存 ----------------

    def _add_point(self, x, y, auto):
        speed = self.get_parameter('default_speed').value
        self.points.append((round(x, 3), round(y, 3), speed))
        tag = '自动' if auto else '手动'
        print(f'[{tag}] #{len(self.points)}: x={x:.2f} y={y:.2f}')

    def save(self):
        if not self.points:
            print('没有路点, 不保存')
            return
        pts = [{'x': x, 'y': y, 'speed': s} for x, y, s in self.points]
        # 末点停车
        pts[-1]['speed'] = 0.0
        lat0, lon0, alt0 = self.origin
        doc = {
            'origin': {'lat': round(lat0, 8), 'lon': round(lon0, 8),
                       'alt': round(alt0, 2)},
            'waypoints': pts,
        }
        path = self.get_parameter('output').value
        header = (
            '# 路点文件 (waypoint_recorder 生成)\n'
            '# x/y: 局部ENU [m], 原点见下 origin 字段\n'
            '# !!! 把 origin.lat/lon 抄到 vehicle.yaml 的 ins_nav_node 节:\n'
            f'#     origin_lat: {lat0:.8f}\n'
            f'#     origin_lon: {lon0:.8f}\n'
            '# 这样每次启动原点一致, 路点不漂移\n')
        with open(path, 'w') as f:
            f.write(header)
            yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
        print(f'已保存 {len(pts)} 个路点 → {path}')
        print(f'origin_lat: {lat0:.8f}  origin_lon: {lon0:.8f}  (请固化到 vehicle.yaml)')

    # ---------------- 投影 (与 ins_nav_node 同一公式) ----------------

    def _project(self, lat, lon):
        zone = self.get_parameter('utm_zone').value
        lat0, lon0, _ = self.origin
        x0, y0 = self._ll_to_utm(lat0, lon0, zone)
        x, y = self._ll_to_utm(lat, lon, zone)
        return x - x0, y - y0

    def _ll_to_utm(self, lat, lon, zone):
        a = 6378137.0; f = 1.0/298.257223563; k0 = 0.9996
        e = math.sqrt(2*f - f*f)
        lat_r = math.radians(lat); lon_r = math.radians(lon)
        lon0 = math.radians(zone * 6 - 183)
        N = a / math.sqrt(1 - e*e * math.sin(lat_r)**2)
        T = math.tan(lat_r)**2
        C = e*e/(1-e*e) * math.cos(lat_r)**2
        A = (lon_r - lon0) * math.cos(lat_r)
        M = a * ((1 - e*e/4 - 3*e**4/64) * lat_r
                 - (3*e*e/8 + 3*e**4/32) * math.sin(2*lat_r)
                 + (15*e**4/256) * math.sin(4*lat_r))
        x = k0*N*(A + (1-T+C)*A**3/6 + (5-18*T+T**2+72*C)*A**5/120) + 500000
        y = k0*(M + N*math.tan(lat_r)*(A**2/2 + (5-T+9*C+4*C**2)*A**4/24
                 + (61-58*T+T**2+600*C)*A**6/720))
        if not self.get_parameter('northern').value:
            y += 10000000
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        node.run()
    except KeyboardInterrupt:
        node.save()
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
