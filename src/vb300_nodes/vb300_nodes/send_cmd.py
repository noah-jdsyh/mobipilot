#!/usr/bin/env python3
"""
send_cmd  —  命令行底盘控制工具
================================
用法:
  ros2 run vb300_nodes send_cmd --throttle 30          # 油门30%
  ros2 run vb300_nodes send_cmd --steer 0.1            # 右转0.1rad
  ros2 run vb300_nodes send_cmd --brake 100            # 急刹
  ros2 run vb300_nodes send_cmd --gear 1 --throttle 20 # D档+油门20%
  ros2 run vb300_nodes send_cmd --auto true            # 进入自动驾驶模式
  ros2 run vb300_nodes send_cmd --epb false            # 拉紧EPB
  ros2 run vb300_nodes send_cmd --stop                 # 停车(油门0+制动100)
  ros2 run vb300_nodes send_cmd --status               # 查看当前底盘状态
  ros2 run vb300_nodes send_cmd --watch                # 持续监听底盘状态

  可组合任意参数, 未指定的保持上次值
"""

import argparse
import rclpy
from rclpy.node import Node
from vb300_interfaces.msg import ChassisCmd, ChassisState


class ChassisCommander(Node):
    def __init__(self):
        super().__init__('chassis_commander')
        self.cmd_pub = self.create_publisher(ChassisCmd, '/cmd', 10)

    def send(self, throttle=None, steer=None, brake=None, gear=None,
             auto=None, epb=None):
        """发送控制指令, 未指定的字段保持上次值"""
        # 获取上次的指令值（如果存在）
        if not hasattr(self, '_last_cmd'):
            self._last_cmd = ChassisCmd()
            self._last_cmd.gear = 255  # 无效值
        
        cmd = ChassisCmd()
        cmd.steer_angle  = float(steer) if steer is not None else self._last_cmd.steer_angle
        cmd.throttle_pct = float(throttle) if throttle is not None else self._last_cmd.throttle_pct
        cmd.brake_pct    = float(brake) if brake is not None else self._last_cmd.brake_pct
        cmd.gear         = int(gear) if gear is not None else self._last_cmd.gear
        cmd.auto_enable  = bool(auto) if auto is not None else self._last_cmd.auto_enable
        cmd.epb_release  = bool(epb) if epb is not None else self._last_cmd.epb_release
        
        # 保存本次指令
        self._last_cmd = cmd

        # 发布3次确保底盘收到
        for _ in range(3):
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        # 打印发送内容
        parts = []
        if steer is not None: parts.append(f"转向={steer}rad({steer*57.3:.1f}°)")
        if throttle is not None: parts.append(f"油门={throttle}%")
        if brake is not None: parts.append(f"制动={brake}%")
        if gear is not None: parts.append(f"档位={'NDR'[gear] if gear<3 else '?'}")
        if auto is not None: parts.append(f"自动={'开' if auto else '关'}")
        if epb is not None: parts.append(f"EPB={'释放' if epb else '拉紧'}")
        print(f"✅ 已发送: {', '.join(parts)}")


def main():
    parser = argparse.ArgumentParser(description='VB300 底盘命令行控制工具')
    parser.add_argument('--throttle', type=float, help='油门 [0-100] %%')
    parser.add_argument('--steer', type=float, help='前轮转角 [rad], 左正右负, 最大±0.5585 (32°)')
    parser.add_argument('--brake', type=float, help='制动 [0-100] %%')
    parser.add_argument('--gear', type=int, choices=[0,1,2], help='档位 0=N 1=D 2=R')
    parser.add_argument('--auto', type=str, choices=['true','false'], help='自动驾驶使能')
    parser.add_argument('--epb', type=str, choices=['true','false'], help='EPB释放(true)/拉紧(false)')
    parser.add_argument('--stop', action='store_true', help='紧急停车(油门0+制动100)')
    parser.add_argument('--status', action='store_true', help='查看当前底盘状态')
    parser.add_argument('--watch', action='store_true', help='持续监听底盘状态')
    parser.add_argument('--init', action='store_true', help='初始化: D档+自动驾驶使能+EPB释放')
    parser.add_argument('--hold', type=float, default=0, help='持续发送指令的秒数 (默认0, 即发送3次后退出)')

    args = parser.parse_args()

    rclpy.init()
    node = ChassisCommander()

    if args.status:
        # 订阅一次状态
        print("等待底盘状态...")
        state = None
        def cb(msg): nonlocal state; state = msg
        sub = node.create_subscription(ChassisState, '/state', cb, 1)
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.1)
            if state:
                print(f"车速: {state.speed:.2f} m/s | 转角: {state.steer_angle:.3f} rad "
                      f"| 制动: {state.brake_pressure:.0f}% | 档位: {state.gear} "
                      f"| 自动: {state.auto_active} | 电机就绪: {state.motor_ready} | EPB: {state.epb_applied} "
                      f"| 电量: {state.battery_soc:.0f}% {state.battery_voltage:.0f}V {state.battery_current:.0f}A "
                      f"| 温度: {state.battery_temp:.0f}°C "
                      f"| 高压: {state.high_voltage_on} 充电: {state.is_charging} BMS故障: {state.bms_error} "
                      f"| 接管: {state.steer_takenover} "
                      f"| 故障: EPS={state.eps_fault} EHB={state.ehb_fault_level}")
                break
        else:
            print("⚠️  未收到底盘状态, 检查 CAN 是否连接")
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.watch:
        print("持续监听底盘状态 (Ctrl+C 退出)...")
        def cb(msg):
            print(f"\r[{node.get_clock().now().to_msg().sec%1000:03d}] "
                  f"速={msg.speed:.2f}m/s 转={msg.steer_angle:.3f}rad "
                  f"制={msg.brake_pressure:.0f}% 档={msg.gear} "
                  f"自动={msg.auto_active} 电机就绪={msg.motor_ready} EPB={msg.epb_applied} "
                  f"SOC={msg.battery_soc:.0f}% {msg.battery_voltage:.0f}V "
                  f"接管={msg.steer_takenover} 故障EPS={msg.eps_fault} EHB={msg.ehb_fault_level} "
                  f"高压={msg.high_voltage_on} BMS={msg.bms_error}",
                  end='', flush=True)
        sub = node.create_subscription(ChassisState, '/state', cb, 10)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print()
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.init:
        node.send(throttle=0, steer=0, brake=0, gear=1, auto='true', epb='true')
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.stop:
        node.send(throttle=0, brake=100)
        node.destroy_node()
        rclpy.shutdown()
        return

    # 普通指令
    auto_val = None
    if args.auto is not None:
        auto_val = args.auto.lower() == 'true'
    epb_val = None
    if args.epb is not None:
        epb_val = args.epb.lower() == 'true'

    if args.hold > 0:
        print(f"持续发送指令 {args.hold}s, 按 Ctrl+C 停止...")
        import time
        start = time.time()
        try:
            while time.time() - start < args.hold:
                node.send(
                    throttle=args.throttle,
                    steer=args.steer,
                    brake=args.brake,
                    gear=args.gear,
                    auto=auto_val,
                    epb=epb_val,
                )
                time.sleep(0.1)  # 10Hz
        except KeyboardInterrupt:
            print("\n停止发送")
    else:
        node.send(
            throttle=args.throttle,
            steer=args.steer,
            brake=args.brake,
            gear=args.gear,
            auto=auto_val,
            epb=epb_val,
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
