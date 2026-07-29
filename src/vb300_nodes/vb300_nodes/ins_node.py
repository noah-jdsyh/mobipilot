#!/usr/bin/env python3
"""
ins_node.py  —  INS5715DAA 完整驱动
=====================================
硬件: 导远 INS5715DAA (NAV3120DAA + IMU5115)
接口: 2×RS232
  RS232-1 (数据口, 出厂 460800): 只输出 BD DB 二进制帧, 不接收 AT 指令
  RS232-2 (指令口, 115200): AT 指令收发 / NMEA(GGA,RMC)输出 / RTCM 差分输入
  (2026-07 实机验证: 数据口发 AT 指令无响应, 指令口收到 AGBDDB0B,...,OK 回包)
协议: BD DB 0B 二进制帧 + AT 指令配置
输出:
  /ins/pose      PoseStamped      100Hz  UTM投影后位姿
  /ins/odom      Odometry         100Hz  速度+角速度
  /ins/fix       NavSatFix         5Hz   原始经纬度(用于调试)
  /ins/status    INSStatus         1Hz   设备状态

使用前:
  1. RS232-1 波特率保持出厂 460800 (如需更改只能用导远上位机, 手册§2.3)
  2. 配置 NTRIP 差分账号(设备内置4G)
  3. 标定杆臂值: AGSETINSANT1,x,y,z
  4. 等待定位指示灯常亮(RTK固定解)
"""

import struct
import time
import threading
import math
import os
import termios
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, UInt8
import serial


# ======================= INS 状态枚举 =======================
class INSStatus:
    UNINIT     = 0    # 未初始化
    ALIGNING   = 1    # 初始对准中
    SINGLE     = 2    # 单点定位
    DGPS       = 3    # 伪距差分
    FLOAT_RTK  = 4    # RTK浮点解
    FIXED_RTK  = 5    # RTK固定解
    INS_ONLY   = 6    # 纯惯导(丢星)
    FAULT      = 7    # 故障


# ======================= 自定义状态消息(轻量) =======================
class INSStatusMsg:
    """INS 状态消息, 用于调试/监控"""
    def __init__(self):
        self.header = Header()
        self.status = 0       # INSStatus 枚举
        self.sat_count1 = 0   # 主天线卫星数
        self.sat_count2 = 0   # 副天线卫星数
        self.age = 0           # 差分延时 [s]
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0


# ======================= INS5715DAA 驱动节点 =======================
class InsNode(Node):
    def __init__(self):
        super().__init__('ins_node')

        # ---- 参数声明 ----
        self.declare_parameter('port_data',  '/dev/ttyUSB1')   # RS232-1 数据口(二进制输出)
        self.declare_parameter('port_diff',  '')               # RS232-2 指令/差分/NMEA口 @115200 (可选, 空=跳过初始化指令)
        self.declare_parameter('baud',        460800)          # RS232-1 出厂波特率
        self.declare_parameter('output_hz',   100)             # BD DB 0B 频率 (2026-07 实测: 下发200Hz后设备输出异常, 保持100)
        self.declare_parameter('utm_zone',    50)              # UTM 分带
        self.declare_parameter('northern',    True)
        self.declare_parameter('lever_arm_x', 0.0)   # IMU→主天线 X [m] 前正
        self.declare_parameter('lever_arm_y', 0.0)   # IMU→主天线 Y [m] 左正
        self.declare_parameter('lever_arm_z', 0.0)   # IMU→主天线 Z [m] 上正
        self.declare_parameter('origin_lat',  0.0)   # 手动设原点纬度(0=第一帧自动设)
        self.declare_parameter('origin_lon',  0.0)
        self.declare_parameter('origin_alt',  0.0)

        # ---- 发布者 (默认 QoS = RELIABLE, 与订阅侧 planner/control 匹配) ----
        self.pose_pub  = self.create_publisher(PoseStamped, '/ins/pose', 10)
        self.odom_pub  = self.create_publisher(Odometry,     '/ins/odom', 10)
        self.fix_pub   = self.create_publisher(NavSatFix,    '/ins/fix',  10)
        self.status_pub = self.create_publisher(UInt8,       '/ins/status', 10)

        # ---- 内部状态 ----
        self.origin_x = None        # UTM 原点
        self.origin_y = None
        self.origin_z = None
        self.status = INSStatus.UNINIT
        self.status_count = 0
        self.stat_msg = INSStatusMsg()

        # ---- 打开并配置串口 (230400 8N1, raw) ----
        port = self.get_parameter('port_data').value
        baud = self.get_parameter('baud').value
        self.ser_fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        self._config_serial(self.ser_fd, baud)
        self.get_logger().info(f'INS 串口已打开: {port} @ {baud}')
        self.buf = bytearray()
        self._last_data = time.time()   # 最近一次收到串口数据的时间

        # 可选: 打开 RS232-2 指令口 (AT指令下发 / NMEA / RTCM差分注入, 115200)
        diff_port = self.get_parameter('port_diff').value
        self.diff_ser = None
        if diff_port:
            try:
                self.diff_ser = serial.Serial(diff_port, 115200, timeout=0.1)
                self.get_logger().info(f'INS 指令口: {diff_port} @ 115200')
            except Exception as e:
                self.get_logger().warn(f'指令口打开失败: {e}')

        # ---- 启动指令 ----
        # 2026-07 实测: 向指令口下发 AGBDDB0B 输出配置指令(100/200Hz均复现)后,
        # 设备数据口会在数秒内停止输出, 需断电重启恢复。
        # 出厂默认已在 RS232-1 输出 BDDB0B@100Hz, 因此默认不下发任何配置指令。
        # 如确需修改输出配置, 置 send_init_cmd: true (风险自担, 改前请先用上位机确认)
        self.declare_parameter('send_init_cmd', False)
        hz = self.get_parameter('output_hz').value
        if self.get_parameter('send_init_cmd').value and self.diff_ser is not None:
            self._send_cmd(f'AGBDDB0B,0,1,{hz}')     # 使能 BD DB 0B 输出
        else:
            self.get_logger().info(
                '跳过初始化指令(出厂默认已输出 BDDB0B@100Hz)')

        # ---- 定时器 ----
        self.create_timer(0.005, self._poll)         # 串口轮询 200Hz
        self.create_timer(1.0,   self._publish_status) # 状态发布 1Hz
        self.create_timer(5.0,   self._keepalive)     # 心跳 5s

        self.get_logger().info('INS5715DAA 驱动初始化完成')

    # ================== 串口操作 ==================

    def _send_cmd(self, cmd: str):
        """发送 AT 指令到 RS232-2 指令口 (数据口不接收指令)"""
        if self.diff_ser is None:
            return
        full = f'{cmd}\r\n'
        self.diff_ser.write(full.encode())
        self.get_logger().debug(f'TX: {cmd}')

    @staticmethod
    def _config_serial(fd, baud: int):
        """termios 配置串口: 指定波特率, 8N1, raw 模式"""
        attrs = termios.tcgetattr(fd)
        baud_const = getattr(termios, f'B{baud}', termios.B230400)
        attrs[4] = baud_const          # c_ispeed
        attrs[5] = baud_const          # c_ospeed
        attrs[0] = 0                   # c_iflag: raw 输入
        attrs[1] = 0                   # c_oflag: raw 输出
        attrs[3] = 0                   # c_lflag: 非规范模式, 无回显
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[6][termios.VMIN]  = 0    # 非阻塞读: 无数据立即返回
        attrs[6][termios.VTIME] = 1    # 最多等 0.1s, 防止串口断流时整个节点卡死
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def _poll(self):
        """主轮询: 从串口读取并解析帧"""
        try:
            raw = os.read(self.ser_fd, 4096)
            if not raw:
                if time.time() - self._last_data > 2.0:
                    self.get_logger().warn('串口无数据(检查设备/线缆/波特率)',
                                           throttle_duration_sec=5)
                    self.status = INSStatus.UNINIT
                return
            self._last_data = time.time()
            self.buf.extend(raw)
            self._parse_buffer()
        except OSError as e:
            self.get_logger().error(f'串口故障: {e}', throttle_duration_sec=5)
            self.status = INSStatus.FAULT
        except Exception as e:
            self.get_logger().warn(f'解析异常: {e}', throttle_duration_sec=5)

    # BD DB 0B 帧布局 (对照官方驱动 pbox_node/src/protocol/decode_0B.cpp):
    #   帧总长 58 字节, 含 3 字节帧头 BD DB 0B
    #   XOR 校验: byte0..byte56 逐字节异或, 结果应等于 byte57
    FRAME_HEADER = b'\xbd\xdb\x0b'
    FRAME_LEN = 58

    def _parse_buffer(self):
        """从缓冲区提取 BD DB 0B 帧 (总长58字节, 含帧头)"""
        while True:
            idx = self.buf.find(self.FRAME_HEADER)
            if idx < 0:
                # 保留尾部可能是半个帧头的字节, 其余丢弃
                if self.buf.endswith(b'\xbd\xdb'):
                    del self.buf[:-2]
                elif self.buf.endswith(b'\xbd'):
                    del self.buf[:-1]
                else:
                    # AT 指令响应为 ASCII 文本, 打印后清空
                    if self.buf.strip():
                        self.get_logger().info(
                            f'INS响应: {bytes(self.buf).decode(errors="ignore").strip()}',
                            throttle_duration_sec=5)
                    self.buf.clear()
                return
            if idx > 0:
                # 帧头前的杂散字节(可能是 AT 响应文本)
                junk = bytes(self.buf[:idx])
                if junk.strip():
                    self.get_logger().info(
                        f'INS响应: {junk.decode(errors="ignore").strip()}',
                        throttle_duration_sec=5)
                del self.buf[:idx]
            if len(self.buf) < self.FRAME_LEN:
                return

            frame = bytes(self.buf[:self.FRAME_LEN])
            # XOR 校验 byte0..byte56 (含帧头, 与官方驱动一致)
            cs = 0
            for b in frame[:self.FRAME_LEN - 1]:
                cs ^= b
            if cs == frame[self.FRAME_LEN - 1]:
                del self.buf[:self.FRAME_LEN]
                self._parse_ins_frame(frame)
            else:
                del self.buf[0]   # 校验失败, 滑动一字节重新找帧头

    # ================== BD DB 0B 解析 ==================

    def _parse_ins_frame(self, frame: bytes):
        """解析 BD DB 0B 帧 (58字节, 含3字节帧头; 偏移均相对帧起始)

        布局 (对照 pbox_node/src/protocol/decode_0B.cpp):
          0-2   帧头 BD DB 0B
          3/5/7 roll/pitch/yaw   I16 ×360/32768  deg (yaw: 正北起顺时针)
          9/11/13  gyro x/y/z    I16 ×300/32768  deg/s
          15/17/19 acc x/y/z     I16 ×12/32768   g
          21/25  lat/lon        I32 ×1e-7       deg
          29     alt            I32 ×1e-3       m
          33/35/37 vn/ve/vd     I16 ×100/32768  m/s
          39     ins_status     U8  初对准完成标志: bit0=位置 bit1=速度
                                    bit2=姿态 bit3=航向角 (1=完成初对准)
          46/48/50 temp[3]      I16 轮询数据 (含义由 byte56 选择)
          52     gps_ms         U32 0.25ms → 周内秒 = /4000
          56     data_type      U8  0=位置精度 1=速度精度 2=姿态精度
                                    22=温度 32=解状态/卫星数 33=轮速状态
          57     XOR校验 (byte0..56)
        注: 官方驱动在偏移58再读4字节作为 GPS 周, 已超出本帧(58字节),
            属于对后续字节的越界读, 本驱动不解析 GPS 周。
        """
        if len(frame) < self.FRAME_LEN:
            return

        # --- 解包(小端序) ---
        u = struct.unpack_from
        roll_raw    = u('<h', frame, 3)[0]
        pitch_raw   = u('<h', frame, 5)[0]
        yaw_raw     = u('<h', frame, 7)[0]
        gyro_x      = u('<h', frame, 9)[0]
        gyro_y      = u('<h', frame, 11)[0]
        gyro_z      = u('<h', frame, 13)[0]
        lat_raw     = u('<i', frame, 21)[0]
        lon_raw     = u('<i', frame, 25)[0]
        alt_raw     = u('<i', frame, 29)[0]
        vn_raw      = u('<h', frame, 33)[0]
        ve_raw      = u('<h', frame, 35)[0]
        vd_raw      = u('<h', frame, 37)[0]
        status_byte = frame[39]              # ins_status U8
        temp0       = u('<h', frame, 46)[0]  # 轮询数据 (含义由 data_type 选择)
        temp1       = u('<h', frame, 48)[0]
        gps_ms      = u('<I', frame, 52)[0]  # 0.25ms
        data_type   = frame[56]              # 轮询数据类型

        # --- 物理量换算 ---
        roll  = roll_raw  * (360.0 / 32768.0)
        pitch = pitch_raw * (360.0 / 32768.0)
        yaw_d = yaw_raw  * (360.0 / 32768.0)   # 度, 正北顺时针
        lat   = lat_raw  * 1e-7
        lon   = lon_raw  * 1e-7
        alt   = alt_raw  * 1e-3
        vn    = vn_raw   * (100.0 / 32768.0)
        ve    = ve_raw   * (100.0 / 32768.0)
        vd    = vd_raw   * (100.0 / 32768.0)
        gx    = gyro_x   * (300.0 / 32768.0) * math.pi / 180.0  # rad/s
        gy    = gyro_y   * (300.0 / 32768.0) * math.pi / 180.0
        gz    = gyro_z   * (300.0 / 32768.0) * math.pi / 180.0

        # --- 状态判断 ---
        # byte39 ins_status (手册BDDB0B协议): bit0=位置 bit1=速度 bit2=姿态
        # bit3=航向角, 1=完成初对准。位置未对准时经纬度字段输出全0
        # (实测: 桌面静止+单天线时 byte39=0x04 仅姿态对准, lat/lon=0;
        #  需双天线定向+一定运动/RTK 才能完成位置与航向对准)
        # 解状态由轮询帧给出: data_type==32 时 temp0=position_type, temp1=卫星数
        # (NovAtel 风格状态字: 48/49/50=RTK固定解, 32/33/34=RTK浮点解)
        pos_aligned = bool(status_byte & 0x01)
        self.status_count += 1
        if data_type == 32:
            position_type = temp0
            self.stat_msg.sat_count1 = temp1
            if position_type in (48, 49, 50):
                self.status = INSStatus.FIXED_RTK
            elif position_type in (32, 33, 34):
                self.status = INSStatus.FLOAT_RTK
            elif position_type in (16, 17):
                self.status = INSStatus.DGPS
            elif position_type > 0:
                self.status = INSStatus.SINGLE
        if not pos_aligned:
            self.status = INSStatus.ALIGNING   # 位置初对准未完成, 覆盖解状态

        # 位置初对准未完成(经纬度全0)视为无效数据, 不发布
        if not pos_aligned or (lat == 0.0 and lon == 0.0):
            return

        self.stat_msg.lat = lat
        self.stat_msg.lon = lon
        self.stat_msg.alt = alt
        self.stat_msg.age = 0

        # --- 坐标系转换 ---
        if self.origin_x is None:
            if self.get_parameter('origin_lat').value != 0:
                # 手动原点
                self.origin_x, self.origin_y = self._ll_to_utm(
                    self.get_parameter('origin_lat').value,
                    self.get_parameter('origin_lon').value)
                self.origin_z = self.get_parameter('origin_alt').value
            else:
                self.origin_x, self.origin_y = self._ll_to_utm(lat, lon)
                self.origin_z = alt
            self.get_logger().info(f'原点: UTM({self.origin_x:.1f}, {self.origin_y:.1f}) 高度{self.origin_z:.2f}m')

        utm_x, utm_y = self._ll_to_utm(lat, lon)
        enu_x = utm_x - self.origin_x
        enu_y = utm_y - self.origin_y
        enu_z = alt - self.origin_z

        # --- Yaw 转换: 协议航向为正北起顺时针(度) → ENU 东起逆时针(rad) ---
        #     yaw_enu = π/2 − heading, wrap 到 [-π, π]
        yaw_rad = math.pi / 2.0 - yaw_d * math.pi / 180.0
        yaw_rad = (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        roll_rad = roll * math.pi / 180.0
        pitch_rad = pitch * math.pi / 180.0

        now = self.get_clock().now().to_msg()

        # --- 发布 Pose ---
        pose = PoseStamped(header=Header(stamp=now, frame_id='map'))
        pose.pose.position = Point(x=enu_x, y=enu_y, z=enu_z)
        pose.pose.orientation = self._rpy_to_quat(roll_rad, pitch_rad, yaw_rad)
        self.pose_pub.publish(pose)

        # --- 发布 Odometry ---
        odom = Odometry(header=Header(stamp=now, frame_id='map'),
                        child_frame_id='base_link')
        odom.pose.pose = pose.pose
        # child_frame_id=base_link: 线速度给前向合速度(低速够用), 不再塞 ENU 分量
        odom.twist.twist = Twist(
            linear=Vector3(x=math.hypot(ve, vn), y=0.0, z=0.0),
            angular=Vector3(x=gx, y=gy, z=gz))
        self.odom_pub.publish(odom)

        # --- 发布 NavSatFix(调试用, 5Hz降频) ---
        if self.status_count % 40 == 0:   # 200Hz→5Hz
            fix = NavSatFix(header=Header(stamp=now, frame_id='gnss'))
            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = alt
            ss = NavSatStatus()
            ss.status = NavSatStatus.STATUS_GBAS_FIX if self.status >= 4 else NavSatStatus.STATUS_FIX
            ss.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_GALILEO | NavSatStatus.SERVICE_COMPASS
            fix.status = ss
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.fix_pub.publish(fix)

    # ================== 坐标转换 ==================

    def _ll_to_utm(self, lat: float, lon: float) -> tuple:
        """WGS84 → UTM (简化公式, 误差 <0.1m/10km)"""
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
        """RPY → Quaternion"""
        cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
        cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
        cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
        return Quaternion(
            x=sr*cp*cy - cr*sp*sy, y=cr*sp*cy + sr*cp*sy,
            z=cr*cp*sy - sr*sp*cy, w=cr*cp*cy + sr*sp*sy)

    # ================== 状态发布 ==================

    def _publish_status(self):
        """1Hz 发布设备状态"""
        msg = UInt8(data=self.status)
        self.status_pub.publish(msg)

        status_names = {0:'UNINIT',1:'ALIGNING',2:'SINGLE',3:'DGPS',
                        4:'FLOAT_RTK',5:'FIXED_RTK',6:'INS_ONLY',7:'FAULT'}
        self.get_logger().info(
            f'INS状态: {status_names.get(self.status,"?")}  '
            f'LLH({self.stat_msg.lat:.7f},{self.stat_msg.lon:.7f},{self.stat_msg.alt:.1f})',
            throttle_duration_sec=5)

    def _keepalive(self):
        """5s 心跳, 检测串口是否正常"""
        pass  # raw fd 无需检测

    # ================== 清理 ==================

    def destroy_node(self):
        if hasattr(self, 'ser_fd'):
            os.close(self.ser_fd)
        if hasattr(self, 'diff_ser') and self.diff_ser:
            self.diff_ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = InsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
