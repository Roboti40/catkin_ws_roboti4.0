import serial
import time

def read_from_serial():
    # Setup serial connection
    # Make sure to replace '/dev/ttyUSB0' with the actual serial port your Arduino is connected to.
    ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:  # Check if line is not empty
                    print("Received data:", line)
                    try:
                        # Assuming the format is "speed,position"
                        speed, position = map(float, line.split(','))
                        print(f"Speed: {speed} m/s, Encoder Position: {position} ticks")
                    except ValueError:
                        print("Error: Received data is not in the expected format")
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        ser.close()

if __name__ == '__main__':
    print("Starting serial communication...")
    read_from_serial()

