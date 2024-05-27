#!/usr/bin/env python
import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
import time

def setup_motor(serial_number, channel):
    motor = DCMotor()
    motor.setDeviceSerialNumber(serial_number)
    motor.setChannel(channel)
    motor.openWaitForAttachment(5000)
    return motor

# ROS node initialization (not strictly necessary if not using ROS features)
rospy.init_node('motor_test')

# Create and setup motors
dcMotor1 = setup_motor(487541, 0)
dcMotor2 = setup_motor(487541, 1)
dcMotor3 = setup_motor(487736, 0)
dcMotor4 = setup_motor(487736, 1)

try:
    # Test sequence
    while True:
        # Set each motor to a different velocity to test them individually
        dcMotor3.setTargetVelocity(-1)  #frontright
        dcMotor4.setTargetVelocity(1)  #frontleft
        dcMotor1.setTargetVelocity(1)  #backleft
        dcMotor2.setTargetVelocity(-1) #backright
        
        
        time.sleep(5)  # run for 5 seconds

        # Stop all motors
        dcMotor1.setTargetVelocity(0)
        dcMotor2.setTargetVelocity(0)
        dcMotor3.setTargetVelocity(0)
        dcMotor4.setTargetVelocity(0)
        time.sleep(2)  # stop for 2 seconds

        # Change velocities or add more sequences as needed

except KeyboardInterrupt:
    # Close your Phidgets once the program is done.
    dcMotor1.setTargetVelocity(0)
    dcMotor2.setTargetVelocity(0)
    dcMotor3.setTargetVelocity(0)
    dcMotor4.setTargetVelocity(0)
    dcMotor1.close()
    dcMotor2.close()
    dcMotor3.close()
    dcMotor4.close()
    print("Motors disconnected and program ended.")

