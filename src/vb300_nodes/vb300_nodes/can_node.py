#!/usr/bin/env python3
"""
can_node.py  —  VB300 底盘 CAN 收发桥
协议: 9条CAN报文 (29-bit 扩展帧)
发行: ACU_General (0xCF008FB, 20ms),  ACU_1_General (0x18EFFFFB, 1s)
收宿: EPS/MCU/EHB/EPB/BMS 状态反馈
输出: /state (ChassisState)  @50Hz
输入: /cmd  (ChassisCmd)     @50Hz
"""

import struct
import time
import threading
import math
import rclpy
from rclpy.node import Node
from vb300_interfaces.msg import ChassisCmd, ChassisState
import socket
import can   # python-can 库


# ========== CAN ID 定义 ==========
ID_CMD             = 0x0CF008FB   # ACU_General        发 → (去掉29bit标识位)
ID_CMD_RAW         = 0xCF008FB    # 实际29bit ID
ID_LIGHT           = 0x18EFFFFB   # ACU_1_General      发 → (近光灯)
ID_EPS             = 0x18FB62F7   # EPS_ACU            收 ←
ID_MCU1            = 0x18F900EF   # MCU_1_General      收 ←
ID_MCU2            = 0x18F560EF   # MCU_2_General      收 ←
ID_EPB             = 0x18F53C0C   # EPB_ACU            收 ←
ID_EHB             = 0x18F40010   # EHB_ACU            收 ←
ID_EHB1            = 0x18FBFF10   # EHB_1_ACU          收 ←
ID_BMS             = 0x18FFEFF3   # BMS_ACU            收 ← (以 DBC 为准)


class CanBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')

        # ---- 参数 ----
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('bitrate', 500000)
        # 转向标定: ratio 为方向盘转角/前轮转角(绝对值); invert=-1 可整体取反; offset 补偿机械中位
        self.declare_parameter('steer_ratio', 24.375)   # 方向盘780°→前轮32°
        self.declare_parameter('steer_offset', 0.0)    # [rad], 加到期望前轮转角上
        self.declare_parameter('steer_invert', 1.0)    # 1.0 或 -1.0, 用于翻转方向
        self.declare_parameter('steer_cmd_msb', True)  # True=命令字节大端, False=小端
        self.declare_parameter('debug_can', False)     # 是否打印发送/接收的 CAN 原始字节

        # ---- 状态 ----
        self.state = ChassisState()
        self.cmd: ChassisCmd = None
        self.seq = 0       # 报文计数器 0-3
        self.last_cmd_time = self.get_clock().now()  # 心跳计时
        self.watchdog_timeout = 2.0   # 2s 没收指令→急停

        # ---- SocketCAN ----
        self.can_bus = can.interface.Bus(
            channel=self.get_parameter('can_interface').value,
            interface='socketcan')

        # ---- 发布 /state @50Hz ----
        self.state_pub = self.create_publisher(ChassisState, '/state', 10)
        self.create_timer(0.02, self.publish_state)   # 50Hz

        # ---- 订阅 /cmd ----
        self.create_subscription(ChassisCmd, '/cmd', self.cmd_callback, 10)

        # ---- 发送线程 (20ms周期) ----
        self.cmd_timer = self.create_timer(0.02, self.send_cmd)

        # ---- 接收线程 ----
        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info('CAN 桥已启动')

    def cmd_callback(self, msg: ChassisCmd):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().info(
            f'收到 /cmd: steer={msg.steer_angle:.3f}rad thr={msg.throttle_pct:.0f}% gear={msg.gear} auto={msg.auto_enable}',
            throttle_duration_sec=2)   # 50Hz刷屏限流: 每2秒一条, 防止淹没关键日志

    def publish_state(self):
        self.state_pub.publish(self.state)

    # ===================== 发送 =====================

    def _send_emergency_stop(self):
        """看门狗超时触发: 零油门+满刹+关闭自动驾驶+拉EPB (DLC=8)"""
        self.get_logger().error('看门狗超时! 发送紧急停车', throttle_duration_sec=1)
        byte0 = (7 & 0x7) | (0 << 3) | (0 << 4) | (0 << 5) | (2 << 6)
        steer_raw = 0xFFFF       # 不控制方向盘
        byte1 = 0x64
        byte2 = 0x00
        byte3 = 0x00             # ACU_Fault_Code
        byte4 = (steer_raw >> 8) & 0xFF  # MSB high byte
        byte5 = steer_raw & 0xFF         # MSB low byte
        byte6 = 0x00             # lights+counter
        byte7 = (byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6) & 0xFF
        data = [byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7]  # DLC=8
        msg = can.Message(arbitration_id=ID_CMD_RAW, data=data, is_extended_id=True)
        try:
            self.can_bus.send(msg)
        except Exception:
            pass

    def send_cmd(self):
        """发送 ACU_General 控制帧 (0xCF008FB, 20ms)"""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.watchdog_timeout:
            self._send_emergency_stop()
            return

        if self.cmd is None:
            return

        sr = self.get_parameter('steer_ratio').value
        steer_offset = self.get_parameter('steer_offset').value
        steer_invert = self.get_parameter('steer_invert').value
        steer_cmd_msb = self.get_parameter('steer_cmd_msb').value
        debug_can = self.get_parameter('debug_can').value

        gear_map = {0: 0x0, 1: 0x4, 2: 0x1}
        gear_val = gear_map.get(self.cmd.gear, 0x7)

        auto = 0x1 if self.cmd.auto_enable else 0x0
        epb  = 0x1 if self.cmd.epb_release else 0x2

        # 期望前轮转角(deg) → 加偏移 → 翻转方向 → 换算为方向盘转角(deg)
        desired_tire_deg = self.cmd.steer_angle * 180.0 / math.pi
        corrected_tire_deg = desired_tire_deg * steer_invert + (steer_offset * 180.0 / math.pi)
        steer_wheel_deg = corrected_tire_deg * sr
        steer_raw = int((steer_wheel_deg + 780.0) / 0.1)
        steer_raw = max(0, min(15599, steer_raw))

        throttle_v  = self.cmd.throttle_pct
        throttle_raw = int(throttle_v / 0.4)
        throttle_raw = min(throttle_raw, 250)   # clamp: 100% / 0.4 = 250, 不发协议无效值

        brake_v = self.cmd.brake_pct
        brake_raw = int(brake_v)
        brake_raw = min(brake_raw, 100)   # BrakePressureReq 以 xlsx 为准: factor 1, 0~100; 超程 clamp, 不发 0xFF 放弃制动

        self.seq = (self.seq + 1) & 0x3

        left_sig  = 1 if getattr(self.cmd, 'left_signal', False) else 0
        right_sig = 1 if getattr(self.cmd, 'right_signal', False) else 0
        rev_light = 1 if getattr(self.cmd, 'reversing_light', False) else 0
        horn_val  = 1 if getattr(self.cmd, 'horn', False) else 0
        hazard    = 1 if getattr(self.cmd, 'hazard_warning', False) else 0
        brake_lt  = 1 if getattr(self.cmd, 'braking_light', False) else 0

        # Byte0: Gear(0-2) | Auto(3) | LeftSig(4) | RightSig(5) | EPB(6-7)
        byte0 = ((gear_val & 0x7)
              | ((auto & 0x1) << 3)
              | ((left_sig & 0x1) << 4)
              | ((right_sig & 0x1) << 5)
              | ((epb & 0x3) << 6))

        byte1 = brake_raw & 0xFF            # BrakePressureReq  bit8-15
        byte2 = throttle_raw & 0xFF          # Expect_Pedal_Depth bit16-23
        byte3 = 0x00                         # ACU_Fault_Code    bit24-31

        # Expect_Steering_Angle: bit32-47, 16-bit
        # 实测该底盘 EPS 使用大端(MSB), 协议文档写的小端(LSB)与实际不符
        if steer_cmd_msb:
            byte4 = (steer_raw >> 8) & 0xFF  # high byte bit32-39 → MSB
            byte5 = steer_raw & 0xFF         # low byte  bit40-47 → MSB
        else:
            byte4 = steer_raw & 0xFF         # low byte  bit32-39 → LSB
            byte5 = (steer_raw >> 8) & 0xFF  # high byte bit40-47 → LSB

        # Byte6: RevLight(0) | Horn(1) | Hazard(2) | BrakeLight(3) | Counter(4-7)
        byte6 = ((rev_light & 0x1)
              | ((horn_val & 0x1) << 1)
              | ((hazard & 0x1) << 2)
              | ((brake_lt & 0x1) << 3)
              | ((self.seq & 0xF) << 4))

        # Checksum = sum(byte0..byte6), bit56-63
        byte7 = (byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6) & 0xFF

        data = [byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7]   # DLC=8


        if debug_can:
            self.get_logger().info(
                f'SEND 0x{ID_CMD_RAW:08X} ' + ' '.join(f'{b:02X}' for b in data) +
                f' | steer_raw={steer_raw} sw_deg={steer_wheel_deg:.1f} tire_deg={corrected_tire_deg:.1f}'
            )

        msg = can.Message(arbitration_id=ID_CMD_RAW, data=data, is_extended_id=True)
        try:
            self.can_bus.send(msg)
        except Exception as e:
            self.get_logger().warn(f'CAN 发送失败: {e}', throttle_duration_sec=5)

    # ===================== 接收 =====================

    def recv_loop(self):
        """CAN 接收线程"""
        while rclpy.ok():
            try:
                msg = self.can_bus.recv(timeout=0.05)
                if msg is None:
                    continue
                self.parse_msg(msg)
            except Exception as e:
                self.get_logger().warn(f'CAN 接收异常: {e}', throttle_duration_sec=5)

    def parse_msg(self, msg):
        """根据 CAN ID 解析信号"""
        mid = msg.arbitration_id
        d = msg.data

        if len(d) < 8:
            return

        if mid == ID_EPS:           # 0x18FB62F7 EPS_ACU
            raw = struct.unpack_from('<H', d, 0)[0]    # bytes0-1
            steer_wheel_deg = None
            tire_deg = None
            if raw != 0xFFFF:
                steer_wheel_deg = raw * 0.1 - 780.0
                sr = self.get_parameter('steer_ratio').value
                steer_invert = self.get_parameter('steer_invert').value
                tire_deg = steer_wheel_deg / sr * steer_invert
                self.state.steer_angle = tire_deg * math.pi / 180.0   # → rad
            self.state.steer_takenover = bool(d[2] & 0x01)   # bit16
            self.state.eps_fault = bool(d[2] & 0x02)         # bit17
            # EPS_Control_Model: bit18, 0x00=手动, 0x01=自动 (厂家确认; 协议值表0x01/0x02为文档错误)
            eps_mode = (d[2] >> 2) & 0x01                    # bit18
            self.state.auto_active = bool(eps_mode)          # 1=自动模式
            if self.get_parameter('debug_can').value:
                dbg = f'RECV 0x{mid:08X} ' + ' '.join(f'{b:02X}' for b in d) + f' | steer_raw={raw}'
                if steer_wheel_deg is not None:
                    dbg += f' sw_deg={steer_wheel_deg:.1f} tire_deg={tire_deg:.1f}'
                self.get_logger().info(dbg)

        elif mid == ID_MCU1:        # 0x18F900EF MCU_1_General
            self.state.speed = d[0] / 3.6    # km/h → m/s

        elif mid == ID_MCU2:        # 0x18F560EF MCU_2_General
            gs = d[0] & 0x03
            self.state.gear = {1: 1, 2: 2}.get(gs, 0)
            self.state.motor_ready = bool(d[0] & 0x20)
            # MCU1 已以 20ms 周期提供车速, MCU2 的 byte6 车速不再覆盖

        elif mid == ID_EPB:         # 0x18F53C0C EPB_ACU
            epb_st = (d[4] >> 0) & 0x07  # bit32-34
            self.state.epb_applied = (epb_st == 1)

        elif mid == ID_EHB:         # 0x18F40010 EHB_ACU
            self.state.brake_pressure = float(d[1])   # byte1
            self.state.ehb_fault_level = (d[4] >> 0) & 0x0F  # bit32-35

        elif mid == ID_EHB1:        # 0x18FBFF10 EHB_1_ACU 诊断
            pass  # 仅错误标志, 有需要再处理

        elif mid == ID_BMS:         # 0x18FFEFF3 BMS_ACU (旧版DBC遗留帧)
            # 注: 20260714 版协议已无 BMS 报文, 本车为铅酸电池不上报电量,
            #     以下字段实车读数恒为 0, 属正常现象 (2026-07 与供应商确认)
            self.state.high_voltage_on = bool(d[0] & 0x01)
            self.state.is_charging     = bool(d[0] & 0x02)
            self.state.dcdc_active     = bool(d[0] & 0x04)
            self.state.bms_error       = d[1]

            soc_raw = d[2]
            if soc_raw <= 0xFA:
                self.state.battery_soc = soc_raw * 0.4

            self.state.battery_voltage = float(d[3])

            temp_raw = d[4]
            if temp_raw <= 0xFA:
                self.state.battery_temp = temp_raw - 50.0

            self.state.battery_current = struct.unpack_from('<h', bytes(d[5:7]))[0] * 0.1


def main():
    rclpy.init()
    node = CanBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
