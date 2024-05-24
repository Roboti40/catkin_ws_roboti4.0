#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import tf2_ros
import numpy as np
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class ExtendedScan:

    def __init__(self):
        self.scan_pub = rospy.Publisher('/merged', LaserScan, queue_size=10)
        self.scan1 = None
        self.scan2 = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('/scan1', LaserScan, self.laser_1_callback)
        rospy.Subscriber('/scan2', LaserScan, self.laser_2_callback)

    def laser_1_callback(self, data1):
        self.scan1 = data1
        self.try_merge_scans()

    def laser_2_callback(self, data2):
        self.scan2 = data2
        self.try_merge_scans()

    def try_merge_scans(self):
        if self.scan1 is None or self.scan2 is None:
            return

        newdata = LaserScan()
        newdata.header.stamp = rospy.Time.now()
        newdata.header.frame_id = 'base_link'

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', self.scan2.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return

        # Convert LaserScan to PointCloud2
        pc2_1 = self.laserscan_to_pointcloud(self.scan1, 'base_link')
        pc2_2 = self.laserscan_to_pointcloud(self.scan2, self.scan2.header.frame_id)

        # Transform the second LIDAR's PointCloud2 to the base_link frame
        pc2_2_transformed = do_transform_cloud(pc2_2, transform)

        # Merge the two point clouds
        merged_pc2 = self.merge_pointclouds(pc2_1, pc2_2_transformed)

        # Convert merged PointCloud2 back to LaserScan
        merged_laserscan = self.pointcloud_to_laserscan(merged_pc2)

        self.scan_pub.publish(merged_laserscan)

    def laserscan_to_pointcloud(self, scan, frame_id):
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if r >= scan.range_min and r <= scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y, 0])
            angle += scan.angle_increment
        header = scan.header
        header.frame_id = frame_id
        return pc2.create_cloud_xyz32(header, points)

    def merge_pointclouds(self, cloud1, cloud2):
        points1 = list(pc2.read_points(cloud1, field_names=("x", "y", "z"), skip_nans=True))
        points2 = list(pc2.read_points(cloud2, field_names=("x", "y", "z"), skip_nans=True))
        merged_points = points1 + points2
        header = cloud1.header
        return pc2.create_cloud_xyz32(header, merged_points)

    def pointcloud_to_laserscan(self, cloud):
        ranges = []
        angle_min = -np.pi
        angle_max = np.pi
        angle_increment = np.radians(1.0)  # Adjust the resolution as needed
        num_readings = int((angle_max - angle_min) / angle_increment)
        ranges = [float('inf')] * num_readings

        for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            angle = np.arctan2(y, x)
            index = int((angle - angle_min) / angle_increment)
            if index >= 0 and index < num_readings:
                range_value = np.sqrt(x**2 + y**2)
                if range_value < ranges[index]:
                    ranges[index] = range_value

        laserscan = LaserScan()
        laserscan.header = cloud.header
        laserscan.angle_min = angle_min
        laserscan.angle_max = angle_max
        laserscan.angle_increment = angle_increment
        laserscan.time_increment = 0  # Not calculated here
        laserscan.scan_time = 0  # Not calculated here
        laserscan.range_min = 0.0  # Set according to your LIDAR specs
        laserscan.range_max = 100.0  # Set according to your LIDAR specs
        laserscan.ranges = ranges
        laserscan.intensities = []  # Intensities not used in this example

        return laserscan

if __name__ == '__main__':
    rospy.init_node('extended_scan_publisher')
    rospy.loginfo('Starting extended_scan_publisher node')
    ES = ExtendedScan()
    rospy.spin()
