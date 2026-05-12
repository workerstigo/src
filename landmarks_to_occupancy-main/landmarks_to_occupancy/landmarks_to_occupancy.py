#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LandmarksToOccupancy(Node):
    def __init__(self):
        super().__init__('landmarks_to_occupancy')

        # 發布 OccupancyGrid
        self.pub = self.create_publisher(OccupancyGrid, 'slam_occupancy_grid', 10)

        # 訂閱 VSLAM landmarks cloud
        self.sub = self.create_subscription(
            PointCloud2,
            '/visual_slam/vis/landmarks_cloud',
            self.landmarks_callback,
            10
        )

        # 地圖參數
        self.resolution = 0.05   # 每格 5cm
        self.size_x = 2000        # 2000格 → 100m
        self.size_y = 2000        # 100m

        # 自動計算 origin，讓地圖中心在 (0,0)
        self.origin_x = float(- (self.size_x * self.resolution) / 2)
        self.origin_y = float(- (self.size_y * self.resolution) / 2)

        self.get_logger().info(
            f'Landmarks → OccupancyGrid 節點啟動完成, 地圖範圍 X:[{self.origin_x},{-self.origin_x}] Y:[{self.origin_y},{-self.origin_y}]'
        )

    def landmarks_callback(self, msg: PointCloud2):
        # 初始化地圖 (-1 = 未知)
        grid = -1 * np.ones((self.size_y, self.size_x), dtype=np.int8)

        # 讀取每個 landmark
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # 過濾高度，避免天花板/地板雜訊
            if z < -0.3 or z > 0.5:
                continue

            # 投影到 OccupancyGrid
            mx = int((x - self.origin_x) / self.resolution)
            my = int((y - self.origin_y) / self.resolution)

            if 0 <= mx < self.size_x and 0 <= my < self.size_y:
                grid[my, mx] = 100  # 標記佔用

        # 建立 OccupancyGrid 訊息
        occ = OccupancyGrid()
        occ.header.stamp = self.get_clock().now().to_msg()
        occ.header.frame_id = "map"

        occ.info.resolution = self.resolution
        occ.info.width = self.size_x
        occ.info.height = self.size_y
        occ.info.origin.position.x = float(self.origin_x)
        occ.info.origin.position.y = float(self.origin_y)
        occ.info.origin.orientation.w = 1.0

        occ.data = grid.flatten().tolist()

        # 發布
        self.pub.publish(occ)
        self.get_logger().info(f"發布 OccupancyGrid (點數: {msg.width * msg.height})")

def main(args=None):
    rclpy.init(args=args)
    node = LandmarksToOccupancy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

