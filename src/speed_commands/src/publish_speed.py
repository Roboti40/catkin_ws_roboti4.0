#!/usr/bin/env python3
import serial
import rospy
from std_msgs.msg import Float32MultiArray

def read_from_serial():
    # Setup serial connection
    # Make sure to replace '/dev/ttyACM0' with the actual serial port your Arduino is connected to.
    ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    # Initialize ROS node
    rospy.init_node('encoder_reader', anonymous=True)
    # Initialize ROS publisher
    pub = rospy.Publisher('/speed_new', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    try:
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:  # Check if line is not empty
                    print("Received data:", line)
                    try:
                        # Assuming the format is "speed1,speed2,speed3,speed4"
                        speeds = list(map(float, line.split(',')))
                        if len(speeds) == 4:
                            print(f"Speed 1: {speeds[0]} m/s, Speed 2: {speeds[1]} m/s, Speed 3: {speeds[2]} m/s, Speed 4: {speeds[3]} m/s")
                            # Create a Float32MultiArray message
                            speed_msg = Float32MultiArray(data=speeds)
                            # Publish the speeds
                            pub.publish(speed_msg)
                        else:
                            print("Error: Expected 4 speed values, got", len(speeds))
                    except ValueError:
                        print("Error: Received data is not in the expected format")
            rate.sleep()
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    finally:
        ser.close()

if __name__ == '__main__':
    print("Starting serial communication...")
    read_from_serial()

