#!/usr/bin/env python3
"""
vb300.launch.py  —  VB300 一键启动
启动顺序: CAN → INS → Livox → 感知 → 规划 → 控制
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('vb300_nodes'), 'config')

    # INS 串口设备参数 (可通过命令行覆盖; 默认 udev 别名, 见 /etc/udev/rules.d/99-ins5715.rules)
    ins_port_arg = DeclareLaunchArgument('ins_port', default_value='/dev/ins_data')
    ins_baud_arg = DeclareLaunchArgument('ins_baud', default_value='460800')

    return LaunchDescription([
        ins_port_arg,
        ins_baud_arg,

        # ==== 第0步: 初始化 CAN (sudo ip link set can0 ...) ====
        ExecuteProcess(
            cmd=['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can',
                 'bitrate', '500000'],
            name='can_setup',
            shell=False,
        ),
        ExecuteProcess(
            cmd=['sudo', 'ip', 'link', 'set', 'up', 'can0'],
            name='can_up',
            shell=False,
        ),

        # ==== 1. CAN 收发  Node ====
        Node(
            package='vb300_nodes',
            executable='can_node',
            name='can_bridge',
            parameters=[os.path.join(cfg, 'vehicle.yaml')],
            output='screen',
                    respawn=True,
                    respawn_delay=2.0,
        ),

        # ==== 2. INS 定位链: 官方 pbox_node 解析串口帧 -> ins_nav_node 转换发布 ====
        # pbox_node: INS5715DAA 官方驱动, MsgType=1 发 imu_msgs/Imu(/Ins)+Gnss(/Gnss)
        Node(
            package='pbox_node',
            executable='pbox_pub',
            name='pbox_node',
            parameters=[{
                'MsgType': 1,                       # 1=Asensing自定义msg (imu_msgs)
                'ConnectionType': 0,                # 0=串口
                'UART_Port': LaunchConfiguration('ins_port'),
                'UART_Baudrate': LaunchConfiguration('ins_baud'),
                'USB_LatencyTime': 16,
                'ProtocolType': 0,
                'Grange0B': 300.0,                  # BDDB0B 陀螺量程 (手册 ±300°/s)
                'Arange0B': 12.0,                   # BDDB0B 加表量程 (手册 ±12g)
                'LogLevel': 2,                      # WARNING, 减少日志
            }],
            output='screen',
                    respawn=True,
                    respawn_delay=2.0,
        ),
        # ins_nav_node: /Ins -> UTM投影/坐标换算 -> /ins/nav + /ins/odom + /ins/status
        Node(
            package='vb300_nodes',
            executable='ins_nav_node',
            name='ins_nav_node',
            parameters=[os.path.join(cfg, 'vehicle.yaml')],
            output='screen',
                    respawn=True,
                    respawn_delay=2.0,
        ),

        # ==== 3. Livox MID360 驱动(需先安装 livox_ros_driver2) ====
        # 注意: 请确认 LiDAR IP 和序列号
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_driver',
            parameters=[{
                'xfer_format': 0,           # PointCloud2
                'multi_topic': 1,           # 每个雷达独立 topic
                'data_src': 0,              # 在线模式
                'publish_freq': 10.0,
                'user_config_path': '/home/cabr/vb300_ws/src/livox_ros_driver2/config/MID360_config.json',
            }],
            output='screen',
                    respawn=True,
                    respawn_delay=2.0,
        ),

        # ==== 3.1 静态 TF (左右对称倒置安装, 参数与 vehicle.yaml grid_perception 一致) ====
        # base_link: 后轴中心地面投影, 车头 +x, 左 +y, 上 +z
        # 雷达系约定: M12 出线 = -X。左雷达(.33) Y 朝车头 yaw=+π/2; 右雷达(.34) Y 朝车尾 yaw=-π/2
        # static_transform_publisher 参数顺序: x y z yaw pitch roll 父坐标系 子坐标系
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_livox_left',
            arguments=['1.53', '0.625', '0.445', '1.5708', '0', '3.14159', 'base_link', 'livox_left'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_livox_right',
            arguments=['1.53', '-0.625', '0.445', '-1.5708', '0', '3.14159', 'base_link', 'livox_right'],
        ),

        # ==== 4. 栅格感知 Node ====
        TimerAction(
            period=3.0,    # 等 LiDAR 启动后再启动
            actions=[
                Node(
                    package='vb300_nodes',
                    executable='grid_node',
                    name='grid_perception',
                    parameters=[os.path.join(cfg, 'vehicle.yaml')],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
            ],
        ),

        # ==== 5. 路点规划 Node ====
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='vb300_nodes',
                    executable='planner_node',
                    name='planner_node',
                    parameters=[os.path.join(cfg, 'vehicle.yaml')],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
            ],
        ),

        # ==== 6. 纯跟踪控制 Node ====
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='vb300_nodes',
                    executable='control_node',
                    name='control_node',
                    parameters=[os.path.join(cfg, 'vehicle.yaml')],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                ),
            ],
        ),
    ])
