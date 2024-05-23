#!/usr/bin/env python

import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
import time
from pynput import keyboard

left_joy_hor = left_joy_ver = right_joy_hor = A = B = X = Y = 0

# Create DCMotor objects
dcMotor1 = DCMotor()
dcMotor2 = DCMotor()
dcMotor3 = DCMotor()
dcMotor4 = DCMotor()

# Set device serial numbers and channels
dcMotor1.setDeviceSerialNumber(487541)
dcMotor1.setChannel(0)
dcMotor2.setDeviceSerialNumber(487541)
dcMotor2.setChannel(1)
dcMotor3.setDeviceSerialNumber(487736)
dcMotor3.setChannel(0)
dcMotor4.setDeviceSerialNumber(487736)
dcMotor4.setChannel(1)

# Open the DCMotor objects
dcMotor1.openWaitForAttachment(5000)
dcMotor2.openWaitForAttachment(5000)
dcMotor3.openWaitForAttachment(5000)
dcMotor4.openWaitForAttachment(5000)

# Set initial target velocity to 0
dcMotor1.setTargetVelocity(0)
dcMotor2.setTargetVelocity(0)
dcMotor3.setTargetVelocity(0)
dcMotor4.setTargetVelocity(0)

def on_press(key):
    global dcMotor1, dcMotor2, dcMotor3, dcMotor4

    try:
        if key.char == 'z':
            # Move forward
            dcMotor1.setTargetVelocity(100)
            dcMotor2.setTargetVelocity(-100)
            dcMotor3.setTargetVelocity(-100)
            dcMotor4.setTargetVelocity(100)
        elif key.char == 's':
            # Move backward
            dcMotor1.setTargetVelocity(-100)
            dcMotor2.setTargetVelocity(100)
            dcMotor3.setTargetVelocity(100)
            dcMotor4.setTargetVelocity(-100)
        elif key.char == 'q':
            # Move left
            dcMotor1.setTargetVelocity(100)
            dcMotor2.setTargetVelocity(100)
            dcMotor3.setTargetVelocity(-100)
            dcMotor4.setTargetVelocity(-100)
        elif key.char == 'd':
            # Move right
            dcMotor1.setTargetVelocity(-100)
            dcMotor2.setTargetVelocity(-100)
            dcMotor3.setTargetVelocity(100)
            dcMotor4.setTargetVelocity(100)
    except AttributeError:
        pass

def on_release(key):
    global dcMotor1, dcMotor2, dcMotor3, dcMotor4

    # Stop the motors when the key is released
    dcMotor1.setTargetVelocity(0)
    dcMotor2.setTargetVelocity(0)
    dcMotor3.setTargetVelocity(0)
    dcMotor4.setTargetVelocity(0)

    if key == keyboard.Key.esc:
        # Stop listener when the esc key is pressed
        return False

# Collect events until released
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# Close the DCMotor objects
dcMotor1.close()
dcMotor2.close()
dcMotor3.close()
dcMotor4.close()

