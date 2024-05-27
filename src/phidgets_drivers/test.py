from Phidget22.Phidget import *
from Phidget22.Net import *
from Phidget22.Devices.DCMotor import *
import time

def main():
	Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)

	dcMotor0 = DCMotor()
	dcMotor1 = DCMotor()

	dcMotor0.setIsRemote(True)
	dcMotor0.setChannel(0)
	dcMotor1.setIsRemote(True)
	dcMotor1.setChannel(1)

	dcMotor0.openWaitForAttachment(20000)
	dcMotor1.openWaitForAttachment(20000)

	dcMotor0.setTargetVelocity(25)
	dcMotor1.setTargetVelocity(25)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	dcMotor0.close()
	dcMotor1.close()

main()
