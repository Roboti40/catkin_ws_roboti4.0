#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class ModelStatesToOdom:
    def __init__(self):
        rospy.init_node('model_states_to_odom', anonymous=True)
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.model_name = rospy.get_param('~model_name', 'robot')  # Name of the model to track

    def callback(self, data):
        try:
            index = data.name.index(self.model_name)
        except ValueError:
            rospy.logwarn("Model name %s not found in /gazebo/model_states", self.model_name)
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Extract pose and twist information
        odom_msg.pose.pose = data.pose[index]
        odom_msg.twist.twist = data.twist[index]

        # Publish the Odometry message
        self.pub.publish(odom_msg)

if __name__ == '__main__':
    try:
        ModelStatesToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

