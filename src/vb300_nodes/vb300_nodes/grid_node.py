#!/usr/bin/env python3
"""
grid_node.py  —  双雷达点云 → 2D 占用栅格
输入: /livox/lidar_192_168_1_33 和 ...34 (PointCloud2, 各自雷达坐标系)
输出: /grid/map  (OccupancyGrid, 50m×50m, 0.1m, frame_id=base_link, 以车为中心)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import math


class GridPerception(Node):
    def __init__(self):
        super().__init__('grid_perception')

        self.declare_parameter('grid_size',  50.0)
        self.declare_parameter('resolution',  0.1)
        self.declare_parameter('ground_z_min', -0.1)
        self.declare_parameter('ground_z_max',  0.2)
        self.declare_parameter('occ_threshold', 2)
        # 车身自遮挡过滤 (base_link 矩形盒, 盒内点不视为障碍)
        # 车身长1.925m(后悬0.22→车头1.7含雷达支架), 宽0.9m+支架, 需留余量
        self.declare_parameter('body_x_min', -0.35)
        self.declare_parameter('body_x_max',  1.75)
        self.declare_parameter('body_y_abs',  0.8)
        self.declare_parameter('lidar_topic_1', '/livox/lidar_192_168_1_33')
        self.declare_parameter('lidar_topic_2', '/livox/lidar_192_168_1_34')
        # 雷达外参 (livox→base_link): x/y/z [m], roll/pitch/yaw [rad] (ZYX 内旋)
        # 实际安装: 左右对称倒置(球罩朝地, roll=π), 见 vehicle.yaml
        for lidar in ('lidar_33', 'lidar_34'):
            for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                self.declare_parameter(f'{lidar}_{k}', 0.0)

        self.grid_size   = self.get_parameter('grid_size').value
        self.resolution  = self.get_parameter('resolution').value
        self.gnd_z_min   = self.get_parameter('ground_z_min').value
        self.gnd_z_max   = self.get_parameter('ground_z_max').value
        self.occ_thresh  = self.get_parameter('occ_threshold').value
        self.body_x_min  = self.get_parameter('body_x_min').value
        self.body_x_max  = self.get_parameter('body_x_max').value
        self.body_y_abs  = self.get_parameter('body_y_abs').value

        self.cells = int(self.grid_size / self.resolution)
        self.half   = self.grid_size / 2.0

        topic1 = self.get_parameter('lidar_topic_1').value
        topic2 = self.get_parameter('lidar_topic_2').value

        # 每个雷达的外参: (旋转矩阵 R[3x3], 平移 t[3]), R = Rz(yaw)Ry(pitch)Rx(roll)
        ext1 = self._build_extrinsic('lidar_33')
        ext2 = self._build_extrinsic('lidar_34')

        self.create_subscription(
            PointCloud2, topic1,
            lambda msg, ext=ext1: self.cloud_cb(msg, ext), 10)
        self.create_subscription(
            PointCloud2, topic2,
            lambda msg, ext=ext2: self.cloud_cb(msg, ext), 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/grid/map', 10)

        self.get_logger().info(f'双雷达栅格感知: {topic1}, {topic2}')

    def _build_extrinsic(self, prefix):
        """从参数构建外参: 返回 (R[3x3], t[3]), R = Rz(yaw)Ry(pitch)Rx(roll)"""
        x, y, z, r, p, yw = (self.get_parameter(f'{prefix}_{k}').value
                             for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'))
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(yw), math.sin(yw)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        R = Rz @ Ry @ Rx
        t = np.array([x, y, z])
        self.get_logger().info(
            f'{prefix}: t=({x:.3f},{y:.3f},{z:.3f}) rpy=({r:.3f},{p:.3f},{yw:.3f})')
        return R, t

    def cloud_cb(self, msg: PointCloud2, ext):
        points = self.unpack_cloud(msg)
        if len(points) < 100:
            return

        # 刚体变换: 雷达系 → base_link, p_b = R @ p_l + t
        R, t = ext
        pts_b = points @ R.T + t
        xb, yb, zb = pts_b[:, 0], pts_b[:, 1], pts_b[:, 2]

        # 车身自遮挡过滤: 盒内点(车身/支架/货箱)不视为障碍
        body = (xb > self.body_x_min) & (xb < self.body_x_max) & (np.abs(yb) < self.body_y_abs)

        # 地面/高度带之外的点投影为障碍
        mask = ((zb <= self.gnd_z_min) | (zb >= self.gnd_z_max)) & (~body)

        grid = np.zeros((self.cells, self.cells), dtype=np.int16)

        ix = ((xb[mask] + self.half) / self.resolution).astype(np.int32)
        iy = ((yb[mask] + self.half) / self.resolution).astype(np.int32)
        valid = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
        np.add.at(grid, (iy[valid], ix[valid]), 1)

        occ_data = np.zeros_like(grid, dtype=np.int8)
        occ_data[grid >= self.occ_thresh] = 100
        occ_data[(grid > 0) & (grid < self.occ_thresh)] = 50

        og = OccupancyGrid()
        og.header.stamp = msg.header.stamp
        og.header.frame_id = 'base_link'   # 以车为中心的栅格: 车头 +x, 左 +y
        og.info.resolution = self.resolution
        og.info.width  = self.cells
        og.info.height = self.cells
        og.info.origin = Pose(position=Point(
            x=-self.half, y=-self.half, z=0.0),
            orientation=Quaternion(w=1.0))
        og.data = occ_data.flatten().tolist()

        self.grid_pub.publish(og)

    @staticmethod
    def unpack_cloud(msg: PointCloud2):
        """PointCloud2 → (N,3) float32, 按字段表构建结构化 dtype 向量化解包"""
        fmt_map = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
                   5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
        names, formats, offsets = [], [], []
        for f in msg.fields:
            if f.name not in ('x', 'y', 'z'):
                continue
            names.append(f.name)
            formats.append(fmt_map[f.datatype])
            offsets.append(f.offset)
        dt = np.dtype({'names': names, 'formats': formats,
                       'offsets': offsets, 'itemsize': msg.point_step})
        arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        return np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float32)


def main():
    rclpy.init()
    node = GridPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
