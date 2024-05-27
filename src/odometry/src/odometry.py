#!/usr/bin/env python
#see the MATLAB program and the odometry publisher of ROS

#For calculations
import numpy as np
import math as mt
import rospy
import tf

#For ROS
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32MultiArray

speed_received1 = speed_received2 = speed_received3 = speed_received4 = 0

def callback(data):
    global speed_received1, speed_received2, speed_received3
    speed_received1 = data.x
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", speed_received1)
    speed_received2 = data.y
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", speed_received2)
    speed_received3 = data.z
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", speed_received3)
    
def callback_bis(data):
    global speed_received4
    speed_received4 = data.x
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", speed_received4)
    
    
    
def callback_new(data):
    global speed_received1, speed_received2, speed_received3, speed_received4
    rospy.loginfo("Received Float32MultiArray message")
    
    # Process the Float32MultiArray message
    speeds = data.data
    if len(speeds) == 4:
        rospy.loginfo(f"Speed 1: {speeds[0]} m/s, Speed 2: {speeds[1]} m/s, Speed 3: {speeds[2]} m/s, Speed 4: {speeds[3]} m/s")
        speed_received1 = speeds[0]
        speed_received2 = speeds[1]
        speed_received3 = speeds[2]
        speed_received4 = speeds[3]

    else:
        rospy.logwarn("Received array does not contain 4 elements")

rospy.init_node('odometry_subscriber')

#Subscriber
#odom_sub = rospy.Subscriber("/speed", Vector3, callback)
#odom_bis_sub = rospy.Subscriber("/speed_bis", Vector3, callback_bis)
odom_new = rospy.Subscriber("/speed_new", Float32MultiArray, callback_new)


#Publisher
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

odom_broadcaster = tf.TransformBroadcaster()

#See MATLAB Program of the kinematics

#Schema
#  Wheel2-----Wheel1
#    |          |
#    |          |
#    |          |
#    |          |
#  Wheel3-----Wheel4

#Schema
#  ///----------\\\
#    |          |
#    |          |
#    |          |
#    |          |
#  ///----------///


#Initialization of the model
#All the numbers are in meters
wheel_radius =0.0485
robot_width=0.5
robot_length=0.74

#See Modelling and Adaptive Control of an Omni Mecanum Wheeled Robot for the explanation of the value of a, b, l and alpha
#Semi-width
a = robot_width/2
#Semi-length
b = robot_length/2
#length between the center of the robot and a wheel
l = mt.sqrt(a*a + b*b)
#Angle between horizontal and a wheel
alpha = mt.atan(b/a)

#Angle between the width of the robot and the horizontal
th = 0
x = 0.0
y = 0.0


current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    
    
    #Definition of the matrix and multiplication between each one (kinematics is here)
    mat1 = np.array([[mt.sqrt(2)/2,mt.sqrt(2)/2, l*mt.sin(mt.pi/4 - alpha)],[mt.sqrt(2)/2,-     mt.sqrt(2)/2, l*mt.sin(mt.pi/4 - alpha)],[-mt.sqrt(2)/2,-mt.sqrt(2)/2, l*mt.sin(mt.pi/4 - alpha)],[-mt.sqrt(2)/2,mt.sqrt(2)/2, l*mt.sin(mt.pi/4 - alpha)]])
    mat2 = np.array([[mt.cos(th),mt.sin(th),0],[-mt.sin(th),mt.cos(th),0],[0,0,1]])
    J = np.mat(mat1) * np.mat(mat2)
    T_J = np.transpose(J) 
    J1 = np.mat(T_J) * np.mat(J)
    J_1 = np.linalg.inv(J1)
    J_plus = np.mat(J_1) * np.mat(T_J)
    
    
    #Positive value for the clockwise direction and negative for the anti-clockwise
    #Speed of motor 1
    speed1 = speed_received1
    #Speed of motor 2
    speed2 = speed_received2
    #Speed of motor 3
    speed3 = speed_received3
    #Speed of motor 4
    speed4 = speed_received4



    mat_speed = np.array([[speed1],[speed2],[speed3],[speed4]])


    result = -(mt.sqrt(2)/2) * wheel_radius * np.mat(J_plus) * np.mat(mat_speed)

    speed_x = -result[0]
    speed_y = +result[1]
    speed_rot = +result[2]
    
    #rospy.loginfo("result")
    #rospy.loginfo(result)
    #rospy.loginfo("speed_x")
    #rospy.loginfo(speed_x)
    #rospy.loginfo("speed_y")
    #rospy.loginfo(speed_y)
    #rospy.loginfo("speed_rot")
    #rospy.loginfo(speed_rot)
        
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    #delta_x = (vx * cos(th) - vy * sin(th)) * dt
    #delta_y = (vx * sin(th) + vy * cos(th)) * dt
    #delta_th = vth * dt
    delta_x = float(-speed_y) * dt
    delta_y = float(-speed_x) * dt
    delta_th = float(speed_rot) * dt

    x += delta_x
    y += delta_y
    th += delta_th
    

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(speed_x, speed_y, 0), Vector3(0, 0, speed_rot))
    
    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
    

