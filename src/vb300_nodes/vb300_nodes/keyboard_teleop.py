#!/usr/bin/env python3
"""
keyboard_teleop.py  —  键盘遥控底盘测试
========================================
用法:  ros2 run vb300_nodes keyboard_teleop
按键:  W/S 油门加减,  A/D 左转右转,  C 转向回正,  空格急刹,  Q 退出

前提: can_node 必须在运行 (负责 /cmd → CAN 转换)
      ros2 run vb300_nodes can_node
"""

import tty
import select
import sys
import termios
import threading
import rclpy
from rclpy.node import Node
from vb300_interfaces.msg import ChassisCmd


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.cmd_pub = self.create_publisher(ChassisCmd, '/cmd', 10)

        # 控制状态
        self.throttle = 0.0
        self.steer    = 0.0   # 前轮转角 [rad]
        self.brake    = 0.0
        self.gear     = 1     # D档
        self.auto     = False
        self.epb      = True  # 释放驻车
        self.running  = True

        # 参数
        self.throttle_step = 5.0    # 每次 +/- 5%
        self.steer_step    = 0.05   # 每次 +/- 0.05rad (~3°前轮)
        self.brake_force   = 100.0  # 急刹力度

        # 50Hz 循环发布
        self.create_timer(0.02, self.publish_cmd)

        self.get_logger().info('键盘遥控已就绪')
        self.print_help()

    def print_help(self):
        print("""
╔══════════════════════════════════════╗
║      VB300 键盘遥控测试              ║
╠══════════════════════════════════════╣
║  W    油门 +5%                       ║
║  S    油门 -5%                       ║
║  A    左转 (逆时针)                  ║
║  D    右转 (顺时针)                  ║
║  C    转向回正                       ║
║  X    油门归零                       ║
║  空格  急刹 (100%)                    ║
║  R    释放刹车                       ║
║  E    启用/停用 自动驾驶标志         ║
║  P    EPB 驻车拉紧/释放              ║
║  1    D档                            ║
║  2    R档                            ║
║  0    N档                            ║
║  Q    退出                            ║
╚══════════════════════════════════════╝
        """)

    def publish_cmd(self):
        cmd = ChassisCmd()
        cmd.steer_angle  = round(self.steer, 4)
        cmd.throttle_pct = round(self.throttle, 1)
        cmd.brake_pct    = round(self.brake, 1)
        cmd.gear         = self.gear
        cmd.auto_enable  = self.auto
        cmd.epb_release  = self.epb
        self.cmd_pub.publish(cmd)

    def handle_key(self, key: str):
        if key == 'w':
            self.throttle = min(100, self.throttle + self.throttle_step)
            self.brake = 0.0
            print(f"油门: {self.throttle:.0f}%")
        elif key == 's':
            self.throttle = max(0.0, self.throttle - self.throttle_step)
            self.brake = 0.0
            print(f"油门: {self.throttle:.0f}%")
        elif key == 'x':
            self.throttle = 0.0
            self.brake = 0.0
            print(f"油门归零")
        elif key == 'a':   # 左转: 左正右负
            self.steer = min(0.43, self.steer + self.steer_step)
            print(f"转向: {self.steer:.3f} rad ({self.steer*57.3:.1f}°)")
        elif key == 'd':   # 右转
            self.steer = max(-0.43, self.steer - self.steer_step)
            print(f"转向: {self.steer:.3f} rad ({self.steer*57.3:.1f}°)")
        elif key == 'c':   # 转向回正 (限幅截断会导致 a/d 步进累计回不到 0)
            self.steer = 0.0
            print("转向回正")
        elif key == ' ':
            self.brake = self.brake_force
            self.throttle = 0.0
            print(f"急刹! {self.brake:.0f}%")
        elif key == 'r':
            self.brake = 0.0
            print("刹车释放")
        elif key == 'e':
            self.auto = not self.auto
            print(f"自动驾驶标志: {'开' if self.auto else '关'}")
        elif key == 'p':
            self.epb = not self.epb
            print(f"EPB: {'释放' if self.epb else '拉紧'}")
        elif key == '1':
            self.gear = 1
            print("档位: D")
        elif key == '2':
            self.gear = 2
            print("档位: R")
        elif key == '0':
            self.gear = 0
            print("档位: N")
        elif key == 'q':
            self.running = False
            self.throttle = 0.0
            self.brake = 100.0
            self.auto = False
            self.epb = False   # EPB拉紧
            self.publish_cmd()
            print("退出, 已刹车+EPB")

    def keyboard_loop(self):
        """非阻塞键盘读取"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    self.handle_key(key)
                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = KeyboardTeleop()

    # 键盘线程
    kb_thread = threading.Thread(target=node.keyboard_loop)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        kb_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
