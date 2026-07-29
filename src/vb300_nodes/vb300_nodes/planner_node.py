#!/usr/bin/env python3
"""
planner_node.py  —  路点规划 + 停障
输入: /ins/odom  /grid/map
输出: /plan/path  (Path),  /plan/stop  (Bool)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool, Float32
import yaml
import os
import math
import numpy as np


class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('planner_node')

        # ---- 加载路点 ----
        decl = self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').value
        if not wp_file:
            # 默认路径
            from ament_index_python.packages import get_package_share_directory
            wp_file = os.path.join(get_package_share_directory('vb300_nodes'),
                                   'config', 'waypoints.yaml')

        self.waypoints = self.load_waypoints(wp_file)
        self.get_logger().info(f'已加载 {len(self.waypoints)} 个路点')

        # ---- 状态 ----
        self.current_index = 0
        self.pose = None
        self.yaw = 0.0
        self.grid = None    # (cells, cells, origin_x, origin_y, resolution)
        self.grid_time = None
        self.stop_flag = False
        self._block_streak = 0   # 停障时间滤波计数
        self._clear_streak = 0   # 放行时间滤波计数
        self.finished = False

        # ---- 参数 ----
        self.declare_parameter('arrive_radius',  0.5)     # 到达半径 [m]
        self.declare_parameter('stop_lookahead', 3.0)     # 停障前瞻距离 [m]
        self.declare_parameter('stop_width',     1.5)     # 停障检测宽度 [m]
        self.declare_parameter('path_horizon',   4)       # path 包含的未来路点数 (折线, 防切弯)

        # ---- 订阅 ----
        self.create_subscription(Odometry, '/ins/odom', self.odom_cb, 10)
        self.create_subscription(OccupancyGrid, '/grid/map', self.grid_cb, 10)
        self.create_subscription(Bool, '/engage', self.engage_cb, 10)
        self._engage_prev = False

        # ---- 发布 ----
        self.path_pub = self.create_publisher(Path, '/plan/path', 10)
        self.stop_pub = self.create_publisher(Bool, '/plan/stop', 10)
        self.speed_pub = self.create_publisher(Float32, '/plan/target_speed', 10)
        self.create_timer(0.1, self.plan)   # 10Hz

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        wps = data.get('waypoints', [])
        return [(w['x'], w['y'], w.get('speed', 0.5)) for w in wps]

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z)
        # 四元数 → yaw
        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def engage_cb(self, msg: Bool):
        """/engage 上升沿: 解除终点锁存, 开始新一圈 (2026-07-26)"""
        if msg.data and not self._engage_prev:
            if self.finished:
                self.finished = False
                self.get_logger().info('重新使能, 终点锁存解除, 开始新一圈')
        self._engage_prev = msg.data

    def grid_cb(self, msg: OccupancyGrid):
        """将 OccupancyGrid 转为本地缓存"""
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = list(msg.data)
        self.grid = (w, h, ox, oy, res, data)
        self.grid_time = self.get_clock().now()

    def is_occupied(self, bx: float, by: float) -> bool:
        """检 base_link 坐标 (bx,by) 在栅格中是否占用 (栅格以车为中心, 车头+x 左+y)"""
        if self.grid is None:
            return False
        w, h, ox, oy, res, data = self.grid
        ix = int((bx - ox) / res)
        iy = int((by - oy) / res)
        if 0 <= ix < w and 0 <= iy < h:
            return data[iy * w + ix] >= 100   # 100=占用
        return False

    def plan(self):
        if self.pose is None:
            return
        if self.finished:
            stop = Bool(data=True)
            self.stop_pub.publish(stop)
            return

        px, py, _ = self.pose

        # 终点判定: 接近最后一个路点即完成并锁存 (不依赖索引严格推进;
        # 修复纯跟踪在终点前环绕导致永远进不了到达半径的问题 2026-07-24)
        if not self.finished:
            lx, ly, _ = self.waypoints[-1]
            if math.hypot(px - lx, py - ly) < self.get_parameter('arrive_radius').value * 1.5:
                self.finished = True
                self.get_logger().info('\033[92m============ 到达终点, 任务完成, 停车 ============\033[0m')
        if self.finished:
            self.stop_pub.publish(Bool(data=True))
            self.speed_pub.publish(Float32(data=0.0))
            return

        # 索引吸附: 若下一路点比当前目标更近, 说明已越过当前目标, 直接前进索引
        # (修复: 起点距路点1超出到达半径时索引卡死, 导致追赶身后路点 2026-07-24)
        while self.current_index + 1 < len(self.waypoints):
            x0, y0, _ = self.waypoints[self.current_index]
            x1, y1, _ = self.waypoints[self.current_index + 1]
            if math.hypot(px - x1, py - y1) < math.hypot(px - x0, py - y0):
                self.current_index += 1
            else:
                break

        # 路点到达检测
        if self.current_index < len(self.waypoints):
            wx, wy, _ = self.waypoints[self.current_index]
            if math.hypot(px - wx, py - wy) < self.get_parameter('arrive_radius').value:
                self.current_index += 1
                self.get_logger().info(f'到达路点 {self.current_index}/{len(self.waypoints)}')

        if self.current_index >= len(self.waypoints):
            self.finished = True
            self.get_logger().info('全部路点到达, 停车')
            self.stop_pub.publish(Bool(data=True))
            self.speed_pub.publish(Float32(data=0.0))
            return

        # 停障检测: 在 base_link 系下查询 (栅格以车为中心, 车头 +x, 左 +y)
        # 栅格超过1秒未更新视为失效 (防止陈旧障碍帧导致误停车 2026-07-25)
        if self.grid is not None and \
                (self.get_clock().now() - self.grid_time).nanoseconds > 1.0e9:
            self.grid = None

        la = self.get_parameter('stop_lookahead').value
        sw = self.get_parameter('stop_width').value
        blocked = False

        # 沿车头 +x 前瞻采样, 中心 + 左右各 sw/2 (默认0.75m)
        for d in np.linspace(0, la, int(la / 0.2)):
            if (self.is_occupied(d, 0.0)
                    or self.is_occupied(d, sw / 2)
                    or self.is_occupied(d, -sw / 2)):
                blocked = True
                break

        # 时间滤波: 连续3周期(0.3s)检测到障碍才停车; 停车后需连续10周期(1s)
        # 无障才放行 (消除栅格边缘抖动导致的走走停停 2026-07-26)
        if blocked:
            self._block_streak += 1
            self._clear_streak = 0
        else:
            self._clear_streak += 1
            if self._clear_streak >= 10:
                self._block_streak = 0
        self.stop_flag = self._block_streak >= 3

        # --- 发布 Path ---
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        # 当前位置
        ps0 = PoseStamped()
        ps0.header = path.header
        ps0.pose.position = Point(x=float(px), y=float(py), z=0.0)
        ps0.pose.orientation = Quaternion(w=1.0)
        path.poses.append(ps0)

        # 当前位置 + 未来 path_horizon 个路点 (折线, 纯跟踪沿折线走防切弯)
        if not blocked:
            n = self.get_parameter('path_horizon').value
            end = min(self.current_index + n, len(self.waypoints))
            for i in range(self.current_index, end):
                wx, wy, _ = self.waypoints[i]
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position = Point(x=float(wx), y=float(wy), z=0.0)
                ps.pose.orientation = Quaternion(w=1.0)
                path.poses.append(ps)

        # 目标速度: 取当前路点的 speed 字段 (供 control 变速, 终点为0)
        cur_speed = self.waypoints[min(self.current_index, len(self.waypoints) - 1)][2]

        self.path_pub.publish(path)
        self.stop_pub.publish(Bool(data=blocked))
        self.speed_pub.publish(Float32(data=float(cur_speed)))


def main():
    rclpy.init()
    node = WaypointPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
