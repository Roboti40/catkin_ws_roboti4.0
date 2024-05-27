#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy


from std_msgs.msg import Float32
from sensor_msgs.msg import Range

from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
import time
from Phidget22.PhidgetException import *


left_joy_hor=left_joy_ver=right_joy_hor=A=B=X=Y = 0
#cmd_vel_x = cmd_vel_y = cmd_vel_rot = 0

#See http://wiki.ros.org/joy and on personal observations
#Schema of the left joystick (Axis name index)
#          1[1]
#          |
#          |
#   1------|------(-1)[0]
#          |
#          |
#         (1)

#Schema of the right joystick (Axis name index)
#          1[4]
#          |
#          |
#   1------|------(-1)[3]
#          |
#          |
#         (1)

#Letters of the controller (Button name index)
#A : [0]
#B : [1]
#X : [2]
#Y : [3]


def callback(data):
    global left_joy_hor, left_joy_ver, right_joy_hor, A, B, X, Y
    left_joy_hor = data.axes[0]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", left_joy_hor)
    left_joy_ver = data.axes[1]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", left_joy_ver)
    right_joy_hor = data.axes[3]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", right_joy_hor)
    A = data.buttons[0]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", A)
    B = data.buttons[1]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", B)
    X = data.buttons[2]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", X)
    Y = data.buttons[3]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", Y)
"""    
def callback_ultrasound(msg):
	print("Distance to object: {:.2f} m - Frame ID: {}".format(msg.range, msg.header.frame_id))

"""
rospy.init_node('joystick_subscriber')


#Subscriber and Publisher node
#Subscriber : do rostopic echo joy to see datas variation
#Do rosmsg show sensor_msgs/Joy to see the structure of the inputs
joystick_sub = rospy.Subscriber("joy", Joy, callback)
#ltrasound_sub = rospy.Subscriber("ultrasound", Range, callback_ultrasound)

#Publisher
#joystick_pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)




while not rospy.is_shutdown():

	dcMotor1 = DCMotor()
	dcMotor2 = DCMotor()
	dcMotor3 = DCMotor()
	dcMotor4 = DCMotor()
	
	dcMotor1.setDeviceSerialNumber(487541)
	dcMotor1.setChannel(0)
	dcMotor2.setDeviceSerialNumber(487541)
	dcMotor2.setChannel(1)
	dcMotor3.setDeviceSerialNumber(487736)
	dcMotor3.setChannel(0)
	dcMotor4.setDeviceSerialNumber(487736)
	dcMotor4.setChannel(1)
	
	
	dcMotor1.openWaitForAttachment(5000)
	dcMotor2.openWaitForAttachment(5000)
	dcMotor3.openWaitForAttachment(5000)
	dcMotor4.openWaitForAttachment(5000)
	
	
	dcMotor1.setTargetVelocity(0)
	dcMotor2.setTargetVelocity(0)
	dcMotor3.setTargetVelocity(0)
	dcMotor4.setTargetVelocity(0)

	#right lateral
	while left_joy_hor < 0 and left_joy_ver == 0 and B == 0:
	 
		#cmd_vel_x = 100 * left_joy_hor
		#cmd_vel_y = 0
		dcMotor1.setTargetVelocity(left_joy_hor)
		dcMotor2.setTargetVelocity(left_joy_hor)
		dcMotor3.setTargetVelocity(-left_joy_hor)
		dcMotor4.setTargetVelocity(-left_joy_hor)
	    
	#left lateral
	while left_joy_hor > 0 and left_joy_ver == 0 and B == 0:
		#cmd_vel_x = 100 * left_joy_hor
		#cmd_vel_y = 0
		dcMotor1.setTargetVelocity(left_joy_hor)
		dcMotor2.setTargetVelocity(left_joy_hor)
		dcMotor3.setTargetVelocity(-left_joy_hor)
		dcMotor4.setTargetVelocity(-left_joy_hor)
		
	#backward
	while left_joy_ver < 0 and left_joy_hor == 0 and B == 0:
		#cmd_vel_x = 0
		#cmd_vel_y = 100 * left_joy_ver
		dcMotor1.setTargetVelocity(-left_joy_ver)
		dcMotor2.setTargetVelocity(left_joy_ver)
		dcMotor3.setTargetVelocity(left_joy_ver)
		dcMotor4.setTargetVelocity(-left_joy_ver)
	    
	#frontward
	while left_joy_ver > 0 and left_joy_hor == 0 and B == 0:
		#cmd_vel_x = 0
		#cmd_vel_y = 100 * left_joy_ver
		dcMotor1.setTargetVelocity(-left_joy_ver)
		dcMotor2.setTargetVelocity(left_joy_ver)
		dcMotor3.setTargetVelocity(left_joy_ver)
		dcMotor4.setTargetVelocity(-left_joy_ver)
	    
	#clockwise
	while right_joy_hor < 0 and B == 0:
		#cmd_vel_x = 0
		#cmd_vel_y = 0
		#cmd_vel_rot = 100 * -(right_joy_hor)      
		dcMotor1.setTargetVelocity(right_joy_hor)
		dcMotor2.setTargetVelocity(right_joy_hor)
		dcMotor3.setTargetVelocity(right_joy_hor)
		dcMotor4.setTargetVelocity(right_joy_hor)  
  
	#anti-clockwise
	while right_joy_hor > 0 and B == 0:
		#cmd_vel_x = 0
		#cmd_vel_y = 0
		#cmd_vel_rot = 100 * -right_joy_hor
		dcMotor1.setTargetVelocity(right_joy_hor)
		dcMotor2.setTargetVelocity(right_joy_hor)
		dcMotor3.setTargetVelocity(right_joy_hor)
		dcMotor4.setTargetVelocity(right_joy_hor)
		
	#diagonal right forward
	while left_joy_hor < 0 and left_joy_ver > 0 and B == 0:
		#cmd_vel_x = 100 * left_joy_ver
		#cmd_vel_y = 100 * left_joy_ver
		dcMotor1.setTargetVelocity(left_joy_hor)
		dcMotor2.setTargetVelocity(0)
		dcMotor3.setTargetVelocity(-left_joy_hor)
		dcMotor4.setTargetVelocity(0)

	#diagonal left forward
	while left_joy_hor > 0 and left_joy_ver > 0 and B == 0:
		#cmd_vel_x = 100 * left_joy_ver
		#cmd_vel_y = 100 * -(left_joy_ver)
		dcMotor1.setTargetVelocity(0)
		dcMotor2.setTargetVelocity(left_joy_hor)
		dcMotor3.setTargetVelocity(0)
		dcMotor4.setTargetVelocity(-left_joy_hor)
	    
	#diagonal left backward
	while left_joy_hor > 0 and left_joy_ver < 0 and B == 0:
		#cmd_vel_x = 100 * left_joy_ver
		#cmd_vel_y = 100 * left_joy_ver
		dcMotor1.setTargetVelocity(left_joy_hor)
		dcMotor2.setTargetVelocity(0)
		dcMotor3.setTargetVelocity(-left_joy_hor)
		dcMotor4.setTargetVelocity(0)
		
	#diagonal right backward
	while left_joy_hor < 0 and left_joy_ver < 0 and B == 0:
		#cmd_vel_x = 100 * left_joy_hor
		#cmd_vel_y = 100 * -(left_joy_hor)
		dcMotor1.setTargetVelocity(0)
		dcMotor2.setTargetVelocity(left_joy_hor)
		dcMotor3.setTargetVelocity(0)
		dcMotor4.setTargetVelocity(-left_joy_hor)
		
	    
		# next, we'll publish the cmd_vel message over ROS
		#joy = Twist()
		    
		#joy = Twist(Vector3(cmd_vel_x, cmd_vel_y, 0), Vector3(0, 0, cmd_vel_rot))

		
		# publish the message
		#joystick_pub.publish(joy)
	    
	    
	    
