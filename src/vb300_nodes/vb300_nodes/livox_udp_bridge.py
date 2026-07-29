#!/usr/bin/env python3
"""
livox_udp_bridge.py — 从 UDP 直接收 Mid-360 点云，发 ROS2 PointCloud2
无需 Livox Viewer 2，无需 livox_ros_driver2
"""

import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np


class LivoxUdpBridge(Node):
    def __init__(self):
        super().__init__('livox_udp_bridge')
        
        # 监听 UDP 端口（和 tcpdump 看到的一致）
        self.declare_parameter('point_port', 56301)
        point_port = self.get_parameter('point_port').value
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', point_port))
        self.sock.settimeout(0.01)
        
        self.pub = self.create_publisher(PointCloud2, '/livox/points', 10)
        self.create_timer(0.1, self.receive)  # 10Hz 收包
        
        self.get_logger().info(f'UDP 点云桥已启动, 监听端口 {point_port}')
        
    def receive(self):
        points_all = []
        try:
            for _ in range(50):  # 每次收最多50个包
                data, addr = self.sock.recvfrom(4096)
                if len(data) < 100:
                    continue
                # Mid-360 点云包: 每个点 9 字节(x,y,z各3字节 + reflect 1字节)
                pts = self.parse_mid360_packet(data)
                if pts is not None:
                    points_all.append(pts)
        except socket.timeout:
            pass
        
        if not points_all:
            return
            
        points = np.vstack(points_all)
        
        # 发布 PointCloud2
        msg = PointCloud2()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='lidar')
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.is_bigendian = False
        msg.is_dense = True
        
        buf = points.astype(np.float32).tobytes()
        msg.data = buf
        msg.row_step = len(buf)
        self.pub.publish(msg)
    
    @staticmethod
    def parse_mid360_packet(data):
        """解析 Mid-360 UDP 点云包"""
        n = len(data)
        if n < 100:
            return None
        
        # Mid-360 点云格式: header(18字节) + point_data
        # 每点: x(int32) y(int32) z(int32) reflect(uint8) = 13 bytes
        header_size = 18
        point_size = 13
        
        if (n - header_size) % point_size != 0:
            # 尝试不同的 header 大小
            header_size = 0
            for hs in [0, 8, 12, 16, 18, 20]:
                if (n - hs) % 13 == 0:
                    header_size = hs
                    break
        
        if header_size is None:
            return None
            
        num_points = (n - header_size) // point_size
        if num_points < 1:
            return None
        
        points = np.zeros((num_points, 4), dtype=np.float32)
        off = header_size
        for i in range(num_points):
            if off + 13 > n:
                break
            try:
                x = struct.unpack_from('<i', data, off)[0]      # mm
                y = struct.unpack_from('<i', data, off+4)[0]    # mm
                z = struct.unpack_from('<i', data, off+8)[0]    # mm
                r = data[off+12]  # reflectivity
                points[i, 0] = x * 0.001  # mm → m
                points[i, 1] = y * 0.001
                points[i, 2] = z * 0.001
                points[i, 3] = float(r)
            except:
                pass
            off += point_size
        
        return points if len(points) > 10 else None


def main():
    rclpy.init()
    node = LivoxUdpBridge()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
