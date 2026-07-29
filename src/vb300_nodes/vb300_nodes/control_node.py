#!/usr/bin/env python3
"""
control_node.py  —  Pure Pursuit 转向 + PID 速度
输入: /ins/odom  /plan/path  /plan/stop  /state
输出: /cmd  (ChassisCmd)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Float32
from vb300_interfaces.msg import ChassisCmd, ChassisState
import math


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('control_node')

        # ---- 车辆参数 ----
        self.declare_parameter('wheelbase',      1.25)    # 轴距 [m]
        self.declare_parameter('max_steer_angle', 0.55)   # 最大前轮转角 [rad] (机械极限0.5585)
        self.declare_parameter('max_speed',       1.39)   # 最高车速 m/s (5km/h)
        self.declare_parameter('min_lookahead',   1.5)    # 最小前视距离 [m]
        self.declare_parameter('lookahead_gain',  0.5)    # 速度增益
        # PID 参数
        self.declare_parameter('kp_speed', 10.0)
        self.declare_parameter('ki_speed', 2.0)
        self.declare_parameter('kd_speed', 0.0)

        # ---- 状态 ----
        self.pose = (0.0, 0.0, 0.0)   # x, y, yaw
        self.cur_speed = 0.0
        self.path = None
        self.stop_flag = False
        self.chassis_state = None
        self.last_odom_time = self.get_clock().now()
        # 定位看门狗: 超过 odom_timeout 秒没收定位→急停
        # 实测园区树荫/楼宇下 GNSS 短时波动 0.3~1.4s, 取 1.5s 区分瞬时波动与真断链
        self.declare_parameter('odom_timeout', 1.5)
        self.odom_timeout = self.get_parameter('odom_timeout').value

        # PID 积分项
        self.speed_error_sum = 0.0
        self.last_error = 0.0
        self._takeover_since = None   # 接管标志消抖计时

        # ---- 自动驾驶开关 ----
        self.engaged = False  # 默认关闭, 需手动发送 /engage

        # ---- 订阅 ----
        self.create_subscription(Odometry, '/ins/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/plan/path', self.path_cb, 10)
        self.create_subscription(Bool, '/plan/stop', self.stop_cb, 10)
        self.create_subscription(ChassisState, '/state', self.state_cb, 10)
        self.create_subscription(Bool, '/engage', self.engage_cb, 10)
        self.create_subscription(Float32, '/plan/target_speed', self.target_speed_cb, 10)
        self.target_speed = None      # 来自 planner 的路点速度, None=用默认值

        # ---- 发布 ----
        self.cmd_pub = self.create_publisher(ChassisCmd, '/cmd', 10)
        self.create_timer(0.02, self.control_loop)   # 50Hz

        self.get_logger().info('Pure Pursuit 控制器已启动')

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z)
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.pose = (p.x, p.y, math.atan2(siny, cosy))
        self.cur_speed = msg.twist.twist.linear.x
        self.last_odom_time = self.get_clock().now()

    def path_cb(self, msg: Path):
        self.path = msg

    def stop_cb(self, msg: Bool):
        self.stop_flag = msg.data

    def target_speed_cb(self, msg: Float32):
        self.target_speed = msg.data

    def state_cb(self, msg: ChassisState):
        self.chassis_state = msg

    def engage_cb(self, msg: Bool):
        """/engage 话题: True=启用自动驾驶, False=关闭"""
        if msg.data and not self.engaged:
            # 启用前检查底盘是否允许
            if self.chassis_state is None:
                self.get_logger().warn('底盘状态未知, 无法启用自动驾驶')
                return
            if not self.chassis_state.motor_ready:
                self.get_logger().warn('电机未就绪, 无法启用自动驾驶')
                return
            self.engaged = True
            self.get_logger().info('✅ 自动驾驶已启用')
        elif not msg.data and self.engaged:
            self.engaged = False
            self.get_logger().info('⛔ 自动驾驶已退出')

    def control_loop(self):
        """主控制循环 50Hz"""
        cmd = ChassisCmd()
        cmd.gear = 1              # D挡
        cmd.auto_enable = self.engaged  # 自动驾驶模式与engaged状态一致
        cmd.epb_release = True

        px, py, yaw = self.pose

        # ---- 紧急停车 ----
        emergency = False
        odom_elapsed = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9

        if odom_elapsed > self.odom_timeout:
            emergency = True
            self.get_logger().error(f'INS 数据中断 {odom_elapsed:.2f}s!', throttle_duration_sec=1)

        # ---- 停障: 刹停等待, 障碍清除后自动继续 (不退出 engaged, 不拉EPB) ----
        # 与急停区分: 停障是常规交互(行人经过), 急停只留给定位中断和硬件故障
        if self.stop_flag and self.engaged:
            cmd.steer_angle = 0.0
            cmd.throttle_pct = 0.0
            cmd.brake_pct = 100.0         # 全力刹停 (60%实测刹不住会蠕行 2026-07-26)
            cmd.auto_enable = True
            self.cmd_pub.publish(cmd)
            self.get_logger().warn('停障等待中 (障碍清除后自动继续)', throttle_duration_sec=5)
            return

        if self.chassis_state is not None:
            # steer_takenover 标志实测语义存疑: 转向执行期间会置位, 但并不影响
            # 底盘执行指令(2026-07 实车多次验证), 且遥控器关机时也会锁存。
            # 因此降级为告警日志, 不作为急停触发 (2026-07-24 实车决定,
            # 真正的人工接管保护依赖: 遥控器掌控 + CAN看门狗 + 停障)。
            if self.engaged and self.chassis_state.steer_takenover:
                self.get_logger().warn('EPS人工干预标志置位 (已降级为提示)',
                                       throttle_duration_sec=10)
            if self.chassis_state.ehb_fault_level > 0:
                emergency = True
                self.get_logger().error(f'EHB故障 等级{self.chassis_state.ehb_fault_level}', throttle_duration_sec=1)
            if self.chassis_state.eps_fault:
                emergency = True
                self.get_logger().error('EPS转向故障!', throttle_duration_sec=1)
            # 铅酸电池不支持SOC/温度检测，跳过电池检查
            # if self.chassis_state.battery_soc < 5.0:
            #     emergency = True
            #     self.get_logger().error(f'电量过低 {self.chassis_state.battery_soc:.0f}%!', throttle_duration_sec=1)
            # if self.chassis_state.battery_temp > 70.0:
            #     emergency = True
            #     self.get_logger().error(f'电池过热 {self.chassis_state.battery_temp:.0f}°C!', throttle_duration_sec=1)

        if emergency:
            self.engaged = False
            cmd.steer_angle = 0.0
            cmd.throttle_pct = 0.0
            cmd.brake_pct = 100.0
            cmd.epb_release = False
            cmd.auto_enable = False
            self.cmd_pub.publish(cmd)
            return

        # ---- 未启用自动驾驶 → 上电自举序列 ----
        # 底盘联锁: EPB释放需自动模式(实测), 电机就绪需EPB释放, engage需电机就绪
        # 因此未使能时也保持自动模式+N档+零油门, 让EPB自动释放完成就绪
        if not self.engaged:
            cmd.steer_angle = 0.0
            cmd.throttle_pct = 0.0
            cmd.brake_pct = 0.0
            cmd.gear = 0              # N档, 不会动
            cmd.auto_enable = True    # 自动模式才能释放EPB (实车验证)
            cmd.epb_release = True
            self.cmd_pub.publish(cmd)
            return

        # ---- 正常控制 ----
        if self.path is None or len(self.path.poses) < 2:
            # 没有路径 → 停车
            cmd.steer_angle = 0.0
            cmd.throttle_pct = 0.0
            cmd.brake_pct = 10.0
            self.cmd_pub.publish(cmd)
            return

        # --- Pure Pursuit 转向 ---
        wb = self.get_parameter('wheelbase').value
        ld_base = self.get_parameter('min_lookahead').value
        ld_gain = self.get_parameter('lookahead_gain').value
        Ld = ld_base + ld_gain * self.cur_speed   # 前视距离

        # 找 path 上距离车辆 > Ld 的第一个路点
        target = None
        for ps in self.path.poses:
            wx = ps.pose.position.x
            wy = ps.pose.position.y
            if math.hypot(wx - px, wy - py) > Ld:
                target = (wx, wy)
                break

        if target is None and len(self.path.poses) > 1:
            # 前视不够远, 取最后一个路点
            ps = self.path.poses[-1]
            target = (ps.pose.position.x, ps.pose.position.y)

        if target:
            tx, ty = target
            alpha = math.atan2(ty - py, tx - px) - yaw
            steer = math.atan2(2.0 * wb * math.sin(alpha), Ld)
            max_st = self.get_parameter('max_steer_angle').value
            cmd.steer_angle = max(-max_st, min(max_st, steer))

        # --- PID 速度控制 ---
        # 目标速度: 优先用 planner 下发的路点速度 (含终点0), 否则默认0.5
        target_speed = self.target_speed if self.target_speed is not None else 0.5
        vel_err = target_speed - self.cur_speed

        kp = self.get_parameter('kp_speed').value
        ki = self.get_parameter('ki_speed').value
        self.speed_error_sum += vel_err * 0.02   # dt=20ms
        self.speed_error_sum = max(-10, min(10, self.speed_error_sum))

        throttle = kp * vel_err + ki * self.speed_error_sum
        throttle = max(0.0, min(80.0, throttle))  # 最大80%油门

        cmd.throttle_pct = float(throttle)
        cmd.brake_pct = 0.0

        # 减速制动
        if vel_err < -0.3:
            brake = min(abs(vel_err) * 30, 100.0)
            cmd.brake_pct = float(brake)
            cmd.throttle_pct = 0.0

        cmd.auto_enable = self.engaged
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
