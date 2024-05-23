import serial

def read_from_serial():
    # Setup serial connection
    # Make sure to replace '/dev/ttyACM0' with the actual serial port your Arduino is connected to.
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:  # Check if line is not empty
                    print("Received data:", line)
                    try:
                        # Assuming the format is "speed1,speed2,speed3,speed4"
                        speeds = list(map(float, line.split(',')))
                        if len(speeds) == 4:
                            print(f"Speed 1: {speeds[0]} m/s, Speed 2: {speeds[1]} m/s, Speed 3: {speeds[2]} m/s, Speed 4: {speeds[3]} m/s")
                        else:
                            print("Error: Expected 4 speed values, got", len(speeds))
                    except ValueError:
                        print("Error: Received data is not in the expected format")
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        ser.close()

if __name__ == '__main__':
    print("Starting serial communication...")
    read_from_serial()

