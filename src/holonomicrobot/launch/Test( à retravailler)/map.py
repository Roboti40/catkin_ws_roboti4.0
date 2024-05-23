#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf import transformations
from nav_msgs.msg import MapMetaData

class LidarFusionAndMapping:

    def __init__(self):
        rospy.init_node('lidar_fusion_and_mapping')
        rospy.loginfo('Starting Lidar Fusion and Mapping node')

        # Publishers
        self.scan_pub = rospy.Publisher('/merged', LaserScan, queue_size=0)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        # Subscribers
        rospy.Subscriber('/lidar1', LaserScan, self.laser_1_callback)
        rospy.Subscriber('/lidar2', LaserScan, self.laser_2_callback)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Data Variables
        self.scan1 = None
        self.scan2 = None

    def laser_1_callback(self, data1):
        self.scan1 = data1

    def laser_2_callback(self, data2):
        self.scan2 = data2

    def transform_scan_to_base_link(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                '/base_link', self.scan2.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            scan2_transformed = tf2_geometry_msgs.do_transform_cloud(self.scan2, transform)
            return scan2_transformed
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None

    def publish_merged_scan(self):
        while not rospy.is_shutdown():
            if self.scan1 is not None and self.scan2 is not None:
                merged_scan = self.merge_scans()
                self.scan_pub.publish(merged_scan)
            rospy.sleep(0.1)

    def merge_scans(self):
        merged_scan = LaserScan()

        merged_scan.header.seq = self.scan1.header.seq
        merged_scan.header.stamp = self.scan1.header.stamp
        merged_scan.header.frame_id = '/base_link'

        scan2_transformed = self.transform_scan_to_base_link()

        if scan2_transformed is not None:
            merged_scan.angle_min = self.scan1.angle_min
            merged_scan.angle_max = self.scan1.angle_max + \
                -(self.scan2.angle_min) + self.scan2.angle_max
            merged_scan.angle_increment = self.scan1.angle_increment
            merged_scan.time_increment = self.scan1.time_increment
            merged_scan.scan_time = self.scan1.scan_time
            merged_scan.range_min = self.scan1.range_min
            merged_scan.range_max = self.scan1.range_max

            merged_scan.ranges = self.scan1.ranges + scan2_transformed.ranges
            merged_scan.intensities = [4096] * len(merged_scan.ranges)

        return merged_scan

    def publish_map(self):
        map_metadata = MapMetaData()
        map_metadata.map_load_time = rospy.Time.now()
        map_metadata.resolution = 0.05  # adjust as needed
        map_metadata.width = 100  # adjust as needed
        map_metadata.height = 100  # adjust as needed
        map_metadata.origin.orientation.w = 1.0

        occupancy_grid = OccupancyGrid(header=rospy.Header(frame_id="/map"), info=map_metadata)
        self.map_pub.publish(occupancy_grid)

    def run(self):
        # Run the lidar fusion and mapping
        fusion_thread = threading.Thread(target=self.publish_merged_scan)
        fusion_thread.start()

        # Run the map publishing
        map_thread = threading.Thread(target=self.publish_map)
        map_thread.start()

        fusion_thread.join()
        map_thread.join()

if __name__ == '__main__':
    lidar_fusion_and_mapping = LidarFusionAndMapping()
    lidar_fusion_and_mapping.run()

