#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

def joy_callback(msg):
    # Publish the LaserScan message to the /lidar1/scan topic
    joy_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('joy_publisher')

    # Set up the publishers for Lidar 1 and Lidar 2
    joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)

    # Set up the subscribers for Lidar 1 and Lidar 2
    rospy.Subscriber('/joy0', Joy, joy_callback)

    rospy.spin()
