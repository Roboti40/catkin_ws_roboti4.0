#!/usr/bin/env python3
import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
import serial
import time
import numpy as np
from pynput import keyboard
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf  # Import the tf module

def setup_motor(serial_number, channel):
    motor = DCMotor()
    motor.setDeviceSerialNumber(serial_number)
    motor.setChannel(channel)
    motor.openWaitForAttachment(5000)
    return motor

# ROS node initialization
rospy.init_node('motor_test')
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

# Create and setup motors with the correct mapping
dcMotor1 = setup_motor(487541, 0) # backleft
dcMotor2 = setup_motor(487541, 1) # backright
dcMotor3 = setup_motor(487736, 0) # frontright
dcMotor4 = setup_motor(487736, 1) # frontleft

# Serial communication setup for Arduino (commented out as it's not used)
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Flag to stop motors
stop_flag = False

# Function to send motor commands
def set_motor_speeds(speeds):
    dcMotor1.setTargetVelocity(-speeds[3])  # backleft
    dcMotor2.setTargetVelocity(-speeds[2])  # backright
    dcMotor3.setTargetVelocity(speeds[0])  # frontright
    dcMotor4.setTargetVelocity(speeds[1])  # frontleft

# Function to calculate wheel speeds using the kinematic model
def mecanum_wheel_speeds(vx, vy, omega):
    L = 0.73  # Distance between front and back wheels in meters
    W = 0.52  # Distance between left and right wheels in meters
    R = 0.05  # Wheel radius in meters

    # Kinematic matrix for Mecanum wheels
    mat = np.matrix([
        [1, -1, -(L + W)],
        [1, 1, (L + W)],
        [1, 1, -(L + W)],
        [1, -1, (L + W)]
    ])

    cmd_vel = np.matrix([vx, vy, omega]).T
    wheel_vel = (1 / R) * (mat * cmd_vel).A1
    return wheel_vel.tolist()

# Function to update position based on commanded velocities
def update_position(current_position, vx, vy, omega, dt):
    # Update position
    dx = vx * dt
    dy = vy * dt
    dtheta = omega * dt

    current_position[0] += dx * np.cos(current_position[2]) - dy * np.sin(current_position[2])
    current_position[1] += dx * np.sin(current_position[2]) + dy * np.cos(current_position[2])
    current_position[2] += dtheta

    return current_position

# Function to stop all motors
def stop_motors():
    set_motor_speeds([0, 0, 0, 0])
    print("Motors stopped. Waiting for new coordinates...")

# Function to handle keyboard events
def on_press(key):
    global stop_flag
    if key == keyboard.Key.space:
        stop_motors()
        stop_flag = True

def listen_for_keyboard():
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

# Control algorithm to move to (x, y)
def move_to(x_target, y_target):
    global stop_flag
    # Initialize variables
    current_position = np.array([0.0, 0.0, 0.0])  # x, y, theta
    k_p = 0.01  # Proportional gain (tune this value)
    last_time = time.time()

    while not rospy.is_shutdown():
        # Check for stop flag
        if stop_flag:
            stop_flag = False
            x_target = float(input("Enter new x coordinate: "))
            y_target = float(input("Enter new y coordinate: "))
            continue

        # Calculate time difference
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Calculate error
        error = np.array([x_target, y_target]) - current_position[:2]

        # Calculate control signal (simple proportional controller)
        control_signal = k_p * error

        # Extract desired velocities
        vx = control_signal[0]
        vy = control_signal[1]
        omega = 0  # Assuming no rotation for simplicity

        # Update the current position based on commanded velocities
        current_position = update_position(current_position, vx, vy, omega, dt)

        # Print the current position and velocities
        print(f"Current Position: x={current_position[0]:.2f}, y={current_position[1]:.2f}, theta={current_position[2]:.2f}")
        print(f"Linear Velocities: vx={vx:.2f}, vy={vy:.2f}")

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = current_position[0]
        odom.pose.pose.position.y = current_position[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, current_position[2]))
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        odom_pub.publish(odom)

        # Convert control signal to motor speeds (using kinematics for Mecanum wheels)
        motor_speeds = mecanum_wheel_speeds(vx, vy, omega)

        # Scale motor speeds to be within the range -1.0 to 1.0
        max_speed = max(abs(speed) for speed in motor_speeds)
        if max_speed > 1.0:
            motor_speeds = [speed / max_speed for speed in motor_speeds]

        # Send motor speeds in the correct order: backleft, backright, frontright, frontleft
        set_motor_speeds(motor_speeds)

        # Check if the position is close enough to the target
        if np.linalg.norm(error) < 0.01:
            print("Target reached")
            break

        time.sleep(0.1)  # Control loop delay

if __name__ == '__main__':
    try:
        # Initial target position
        x_target = 0.0
        y_target = -1.0
        listen_for_keyboard()
        move_to(x_target, y_target)  # Example target position (in meters)
    except KeyboardInterrupt:
        print("Program terminated")
    finally:
        set_motor_speeds([0, 0, 0, 0])  # Stop all motors
        # ser.close()  # Commented out as it's not used
        dcMotor1.close()
        dcMotor2.close()
        dcMotor3.close()
        dcMotor4.close()
