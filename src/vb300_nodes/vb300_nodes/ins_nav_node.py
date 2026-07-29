#!/usr/bin/env python3
"""
ins_nav_node.py — 循迹定位转换节点
=====================================
订阅官方驱动 pbox_node 发布的 imu_msgs/Imu (/Ins, BDDB0B 解析结果),
做 UTM 投影 + 坐标/单位换算后, 以自定义消息发布给循迹链路:

  /ins/nav     vb300_interfaces/InsNav   100Hz  自定义循迹定位消息 (对齐后才发)
  /ins/odom    nav_msgs/Odometry         100Hz  兼容 planner/control 现有接口
  /ins/status  std_msgs/UInt8              1Hz  设备状态 (始终发布)

依赖链路: INS5715DAA --RS232--> pbox_node --/Ins--> 本节点
注意: pbox_node 需以 MsgType=1 (Asensing msg) 运行。
姿态按协议原值(前-右-下)只做角度单位换算, 循迹仅用 yaw; yaw 已按
"正北顺时针" → ENU "东起逆时针" 换算 (yaw = π/2 − azimuth)。
"""

import math
import rclpy
from rclpy.node import Node
from imu_msgs.msg import Imu
from vb300_interfaces.msg import InsNav
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Header
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3


# INS 状态枚举 (与 ins_node.py / 既有下游一致)
class INSStatus:
    UNINIT     = 0
    ALIGNING   = 1    # 初对准中
    SINGLE     = 2    # 单点定位
    DGPS       = 3    # 伪距差分
    FLOAT_RTK  = 4    # RTK 浮点解
    FIXED_RTK  = 5    # RTK 固定解
    INS_ONLY   = 6    # 纯惯导
    FAULT      = 7


class InsNavNode(Node):
    def __init__(self):
        super().__init__('ins_nav_node')

        self.declare_parameter('utm_zone',    50)
        self.declare_parameter('northern',    True)
        self.declare_parameter('origin_lat',  0.0)   # 手动原点纬度(0=首个对齐帧自动设)
        self.declare_parameter('origin_lon',  0.0)
        self.declare_parameter('origin_alt',  0.0)
        self.declare_parameter('ins_topic',   '/Ins')

        self.nav_pub    = self.create_publisher(InsNav,  '/ins/nav',    10)
        self.odom_pub   = self.create_publisher(Odometry,'/ins/odom',   10)
        self.status_pub = self.create_publisher(UInt8,   '/ins/status', 10)

        self.create_subscription(
            Imu, self.get_parameter('ins_topic').value, self._on_ins, 10)
        self.create_timer(1.0, self._publish_status)

        self.origin_x = None
        self.origin_y = None
        self.origin_z = None
        self.status = INSStatus.UNINIT

        self.get_logger().info(
            f'ins_nav_node 启动, 订阅 {self.get_parameter("ins_topic").value} (pbox_node MsgType=1)')

    # ================== /Ins 回调 ==================

    def _on_ins(self, msg: Imu):
        m = msg.imu_msg
        aligned = bool(m.ins_status & 0x01)          # bit0: 位置初对准完成

        # --- 解状态映射 (附录一: 48/49/50固定 32/34浮点 16单点 17伪距差分) ---
        pt = m.position_type
        if pt in (48, 49, 50):
            self.status = INSStatus.FIXED_RTK
        elif pt in (32, 33, 34):
            self.status = INSStatus.FLOAT_RTK
        elif pt in (16, 17):
            self.status = INSStatus.DGPS
        elif pt > 0:
            self.status = INSStatus.SINGLE
        if not aligned or (m.latitude == 0.0 and m.longitude == 0.0):
            self.status = INSStatus.ALIGNING
            return          # 位置未对准, 定位字段无效, 不发布

        # --- 原点与 UTM 投影 ---
        if self.origin_x is None:
            if self.get_parameter('origin_lat').value != 0:
                self.origin_x, self.origin_y = self._ll_to_utm(
                    self.get_parameter('origin_lat').value,
                    self.get_parameter('origin_lon').value)
                self.origin_z = self.get_parameter('origin_alt').value
            else:
                self.origin_x, self.origin_y = self._ll_to_utm(m.latitude, m.longitude)
                self.origin_z = m.altitude
            self.get_logger().info(
                f'原点: UTM({self.origin_x:.1f}, {self.origin_y:.1f}) 高度{self.origin_z:.2f}m')

        utm_x, utm_y = self._ll_to_utm(m.latitude, m.longitude)
        x = utm_x - self.origin_x
        y = utm_y - self.origin_y
        z = m.altitude - self.origin_z

        # --- 角度换算: 协议方位角正北顺时针(deg) → ENU 东起逆时针(rad) ---
        yaw = math.pi / 2.0 - math.radians(m.azimuth)
        yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
        roll  = math.radians(m.roll)
        pitch = math.radians(m.pitch)
        yaw_rate = math.radians(m.z_angular_velocity)   # deg/s → rad/s

        now = self.get_clock().now().to_msg()

        # --- 自定义循迹消息 ---
        nav = InsNav()
        nav.header = Header(stamp=now, frame_id='map')
        nav.latitude, nav.longitude, nav.altitude = m.latitude, m.longitude, m.altitude
        nav.x, nav.y, nav.z = x, y, z
        nav.roll, nav.pitch, nav.yaw = roll, pitch, yaw
        nav.speed = math.hypot(m.east_velocity, m.north_velocity)
        nav.vn, nav.ve, nav.vd = m.north_velocity, m.east_velocity, m.ground_velocity
        nav.yaw_rate = yaw_rate
        nav.status = self.status
        nav.position_type = pt
        nav.numsv = m.numsv
        nav.aligned = True
        self.nav_pub.publish(nav)

        # --- 兼容接口 Odometry (planner/control 现有订阅) ---
        odom = Odometry(header=Header(stamp=now, frame_id='map'),
                        child_frame_id='base_link')
        odom.pose.pose.position = Point(x=x, y=y, z=z)
        odom.pose.pose.orientation = self._rpy_to_quat(roll, pitch, yaw)
        odom.twist.twist = Twist(
            linear=Vector3(x=nav.speed, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=yaw_rate))
        self.odom_pub.publish(odom)

    # ================== 状态发布 ==================

    def _publish_status(self):
        self.status_pub.publish(UInt8(data=self.status))
        names = {0:'UNINIT',1:'ALIGNING',2:'SINGLE',3:'DGPS',
                 4:'FLOAT_RTK',5:'FIXED_RTK',6:'INS_ONLY',7:'FAULT'}
        self.get_logger().info(
            f'INS状态: {names.get(self.status, "?")}',
            throttle_duration_sec=5)

    # ================== 工具 ==================

    def _ll_to_utm(self, lat: float, lon: float) -> tuple:
        """WGS84 → UTM (简化公式, 园区尺度误差可忽略)"""
        zone = self.get_parameter('utm_zone').value
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

    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
        cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
        cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
        return Quaternion(
            x=sr*cp*cy - cr*sp*sy, y=cr*sp*cy + sr*cp*sy,
            z=cr*cp*sy - sr*sp*cy, w=cr*cp*cy + sr*sp*sy)


def main(args=None):
    rclpy.init(args=args)
    node = InsNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
