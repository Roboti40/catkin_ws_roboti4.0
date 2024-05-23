#!/usr/bin/env python3
import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
import serial
import time
import numpy as np
from pynput import keyboard
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

def setup_motor(serial_number, channel):
    motor = DCMotor()
    motor.setDeviceSerialNumber(serial_number)
    motor.setChannel(channel)
    motor.openWaitForAttachment(5000)
    return motor

# ROS node initialization
rospy.init_node('motor_test')

# Create and setup motors with the correct mapping
dcMotor1 = setup_motor(487541, 0) # backleft
dcMotor2 = setup_motor(487541, 1) # backright
dcMotor3 = setup_motor(487736, 0) # frontright
dcMotor4 = setup_motor(487736, 1) # frontleft

# Serial communication setup for Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Initialize ROS publishers
pub = rospy.Publisher('/speed_new', Float32MultiArray, queue_size=10)
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

rate = rospy.Rate(10)  # 10 Hz

# Flag to stop motors
stop_flag = False

# Variables for keyboard control
keyboard_control = {'vx': 0.0, 'vy': 0.0, 'omega': 0.0}
target_position = {'x': 0.0, 'y': 0.0}

# Function to send motor commands
def set_motor_speeds(speeds):
    dcMotor1.setTargetVelocity(-speeds[3])  # backleft
    dcMotor2.setTargetVelocity(-speeds[2])  # backright
    dcMotor3.setTargetVelocity(speeds[0])  # frontright
    dcMotor4.setTargetVelocity(speeds[1])  # frontleft

# Function to read encoder values
def read_encoders():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        print(f"Raw data received: '{line}'")  # Debugging statement
        if line:
            try:
                positions = list(map(float, line.split(',')))
                if len(positions) == 4:
                    return positions
                else:
                    print("Error: Expected 4 encoder values, got", len(positions))
            except ValueError:
                print("Error: Received data is not in the expected format")
    return None

# Function to calculate wheel speeds using the kinematic model
def mecanum_wheel_speeds(vx, vy, omega):
    L = 0.74  # Distance between front and back wheels in meters
    W = 0.5  # Distance between left and right wheels in meters
    R = 0.0485  # Wheel radius in meters

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

# Function to calculate the robot's estimated position
def update_position(current_position, encoder_positions, prev_encoder_positions, dt):
    L = 0.73
    W = 0.52
    R = 0.05

    # Calculate wheel velocities (distance per tick, assume ticks/second for velocity)
    wheel_displacements = encoder_positions - prev_encoder_positions

    # Calculate robot velocities
    vx = (wheel_displacements[0] + wheel_displacements[1] + wheel_displacements[2] + wheel_displacements[3]) * (R / 4)
    vy = (-wheel_displacements[0] + wheel_displacements[1] + wheel_displacements[2] - wheel_displacements[3]) * (R / 4)
    omega = (-wheel_displacements[0] + wheel_displacements[1] - wheel_displacements[2] + wheel_displacements[3]) * (R / (4 * (L + W)))

    # Update position
    dx = vx * dt
    dy = vy * dt
    dtheta = omega * dt

    current_position[0] += dx * np.cos(current_position[2]) - dy * np.sin(current_position[2])
    current_position[1] += dx * np.sin(current_position[2]) + dy * np.cos(current_position[2])
    current_position[2] += dtheta

    return current_position, vx, vy, omega

# Function to stop all motors
def stop_motors():
    set_motor_speeds([0, 0, 0, 0])
    print("Motors stopped. Waiting for new commands...")

# Function to handle keyboard events
def on_press(key):
    global stop_flag, target_position
    try:
        if key == keyboard.Key.space:
            stop_motors()
            stop_flag = True
        elif key.char == 'w':
            keyboard_control['vx'] += 0.1
        elif key.char == 's':
            keyboard_control['vx'] -= 0.1
        elif key.char == 'a':
            keyboard_control['vy'] += 0.1
        elif key.char == 'd':
            keyboard_control['vy'] -= 0.1
        elif key.char == 'q':
            keyboard_control['omega'] += 0.1
        elif key.char == 'e':
            keyboard_control['omega'] -= 0.1
        elif key.char == 'x':
            target_position['x'] = float(input("Enter new x coordinate: "))
            target_position['y'] = float(input("Enter new y coordinate: "))
            print(f"New target position set to x: {target_position['x']}, y: {target_position['y']}")
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char == 'w' or key.char == 's':
            keyboard_control['vx'] = 0.0
        elif key.char == 'a' or key.char == 'd':
            keyboard_control['vy'] = 0.0
        elif key.char == 'q' or key.char == 'e':
            keyboard_control['omega'] = 0.0
    except AttributeError:
        pass

def listen_for_keyboard():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

if __name__ == '__main__':
    try:
        listen_for_keyboard()
        current_position = np.array([0.0, 0.0, 0.0])
        prev_encoder_positions = np.array([0.0, 0.0, 0.0, 0.0])
        last_time = time.time()

        while not rospy.is_shutdown():
            encoder_values = read_encoders()
            if encoder_values:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                encoder_positions = np.array(encoder_values)
                current_position, vx, vy, omega = update_position(current_position, encoder_positions, prev_encoder_positions, dt)
                prev_encoder_positions = encoder_positions

                # Publish the encoder values
                speed_msg = Float32MultiArray(data=encoder_values)
                pub.publish(speed_msg)

                # Publish the odometry message
                current_time_ros = rospy.Time.now()
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, current_position[2])

                odom_broadcaster.sendTransform(
                    (current_position[0], current_position[1], 0.),
                    odom_quat,
                    current_time_ros,
                    "base_link",
                    "odom"
                )

                odom = Odometry()
                odom.header.stamp = current_time_ros
                odom.header.frame_id = "odom"
                odom.pose.pose = Pose(Point(current_position[0], current_position[1], 0.), Quaternion(*odom_quat))
                odom.child_frame_id = "base_link"
                odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, omega))

                odom_pub.publish(odom)

                # Control robot using keyboard inputs
                motor_speeds = mecanum_wheel_speeds(keyboard_control['vx'], keyboard_control['vy'], keyboard_control['omega'])
                set_motor_speeds(motor_speeds)

            rate.sleep()
    except KeyboardInterrupt:
        print("Program terminated")
    finally:
        set_motor_speeds([0, 0, 0, 0])  # Stop all motors
        ser.close()
        dcMotor1.close()
        dcMotor2.close()
        dcMotor3.close()
        dcMotor4.close()
