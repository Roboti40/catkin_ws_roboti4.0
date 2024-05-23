from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
import time

#Declare any event handlers here. These will be called every time the associated event occurs.

def main():
	#Create your Phidget channels
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

	
	dcMotor1.setTargetVelocity(0.5)
	dcMotor2.setTargetVelocity(0.5)
	dcMotor3.setTargetVelocity(0.5)
	dcMotor4.setTargetVelocity(0.5)
	#Close your Phidgets once the program is done.
	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	#Close your Phidgets once the program is done.
	dcMotor1.close()

main()
