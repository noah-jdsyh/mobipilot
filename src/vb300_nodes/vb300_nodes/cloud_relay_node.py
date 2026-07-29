#!/usr/bin/env python3
"""
cloud_relay_node.py — 点云 frame_id 转发节点 (仅用于 rviz 可视化)

Livox 驱动对所有雷达共用一个 frame_id (livox_frame), 两台 Mid-360 的点云
无法通过 TF 区分。本节点把两路点云原样转发, 仅改写 frame_id:
  /livox/lidar_192_168_1_33 → /livox/points_left  (frame_id: livox_left)
  /livox/lidar_192_168_1_34 → /livox/points_right (frame_id: livox_right)
配合静态 TF (base_link→livox_left/right), rviz 即可在 base_link 下以
正确姿态显示两台倒置雷达的点云。对感知链路 (grid_node) 无任何影响。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudRelay(Node):
    def __init__(self):
        super().__init__('cloud_relay')
        self.declare_parameter('topic_in_1',  '/livox/lidar_192_168_1_33')
        self.declare_parameter('topic_in_2',  '/livox/lidar_192_168_1_34')
        self.declare_parameter('topic_out_1', '/livox/points_left')
        self.declare_parameter('topic_out_2', '/livox/points_right')
        self.declare_parameter('frame_1', 'livox_left')
        self.declare_parameter('frame_2', 'livox_right')

        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        for i in ('1', '2'):
            tin  = self.get_parameter(f'topic_in_{i}').value
            tout = self.get_parameter(f'topic_out_{i}').value
            frame = self.get_parameter(f'frame_{i}').value
            pub = self.create_publisher(PointCloud2, tout, 10)
            self.create_subscription(
                PointCloud2, tin,
                lambda msg, p=pub, f=frame: self._relay(msg, p, f), sub_qos)
            self.get_logger().info(f'{tin} → {tout} (frame_id={frame})')

    @staticmethod
    def _relay(msg: PointCloud2, pub, frame: str):
        msg.header.frame_id = frame
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
