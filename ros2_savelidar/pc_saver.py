import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d
import struct

class PointCloudSaver(Node):

    def __init__(self):
        super().__init__('pc_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        self.save_pointcloud(msg)

    def save_pointcloud(self, msg):
        points = self.read_points(msg)
        if not points:
            self.get_logger().error('No points found in the point cloud data')
            return

        # Convert points to numpy array
        pc_array = np.array(points)

        # Create Open3D point cloud
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pc_array)

        # Save to PCD file
        o3d.io.write_point_cloud("pointcloud.pcd", pc)
        self.get_logger().info('Point cloud saved to pointcloud.pcd')

    def read_points(self, cloud):
        fmt = self.get_struct_format(cloud.fields, cloud.is_bigendian)
        width, height, point_step, row_step = cloud.width, cloud.height, cloud.point_step, cloud.row_step
        data = cloud.data

        points = []
        for v in range(height):
            for u in range(width):
                offset = v * row_step + u * point_step
                point = struct.unpack_from(fmt, data, offset)
                points.append(point[:3])  # Assuming the first three fields are x, y, z

        return points

    def get_struct_format(self, fields, is_bigendian):
        # Determine the format of the point data
        fmt = '>' if is_bigendian else '<'
        for field in fields:
            if field.datatype == PointField.FLOAT32:
                fmt += 'f'
            elif field.datatype == PointField.FLOAT64:
                fmt += 'd'
            elif field.datatype == PointField.UINT32:
                fmt += 'I'
            elif field.datatype == PointField.INT32:
                fmt += 'i'
            elif field.datatype == PointField.UINT16:
                fmt += 'H'
            elif field.datatype == PointField.INT16:
                fmt += 'h'
            elif field.datatype == PointField.UINT8:
                fmt += 'B'
            elif field.datatype == PointField.INT8:
                fmt += 'b'
            else:
                self.get_logger().error(f"Unsupported PointField datatype: {field.datatype}")

        return fmt

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver = PointCloudSaver()
    rclpy.spin(pointcloud_saver)
    pointcloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
