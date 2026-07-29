"""livox_view.launch.py — 双 Mid-360 点云查看 (驱动 + rviz2)

用法:
    source /opt/ros/humble/setup.bash && source ~/vb300_ws/install/setup.bash
    ros2 launch vb300_nodes livox_view.launch.py          # 驱动 + rviz2
    ros2 launch vb300_nodes livox_view.launch.py rviz:=false  # 只起驱动

话题: /livox/lidar_192_168_1_33, /livox/lidar_192_168_1_34 (frame_id: livox_frame)
前提: 主机网卡已配置 192.168.1.50/24 (雷达 .33/.34 直连/交换机)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_driver',
        parameters=[{
            'xfer_format': 0,           # 0-PointCloud2
            'multi_topic': 1,           # 每个雷达一个话题
            'data_src': 0,              # 在线模式
            'publish_freq': 10.0,
            'frame_id': 'livox_frame',
            'user_config_path': '/home/cabr/vb300_ws/src/livox_ros_driver2/config/MID360_config.json',
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # 静态 TF (与 vehicle.yaml grid_perception 一致, 供 rviz 显示 base_link 系栅格)
    # 参数顺序: x y z yaw pitch roll 父坐标系 子坐标系
    tf_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_livox_left',
        arguments=['1.53', '0.625', '0.445', '1.5708', '0', '3.14159', 'base_link', 'livox_left'],
    )
    tf_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_livox_right',
        arguments=['1.53', '-0.625', '0.445', '-1.5708', '0', '3.14159', 'base_link', 'livox_right'],
    )

    # 点云 frame_id 转发 (livox_frame → livox_left/right), 使 rviz 能按 TF 摆正两台倒置雷达
    relay = Node(
        package='vb300_nodes',
        executable='cloud_relay_node',
        name='cloud_relay',
    )

    return LaunchDescription([rviz_arg, livox_driver, rviz, tf_left, tf_right, relay])
