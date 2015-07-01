from control_api import RoboRoach, time
from control_api import PolarisDriver
from select import select as sel
import numpy as np
import sys

##########################
####### INITIALIZATION
##########################

print("Initializing Polaris... (make sure system is on and connected)")
polaris_driver = PolarisDriver(port='/dev/ttyUSB0')  

polaris_driver.open()

print polaris_driver._apirev()

polaris_driver.initStrayMarkerTracker()
print("Initialized")

print("Initializing RoboRoach... (make sure board is on)")
roboroach = RoboRoach(mac_address="90:59:AF:14:08:E8")

# Standard period is ~19.2ms == ~52Hz
roboroach._set_freq(50)

# Standard pulse width is ~10.4ms
roboroach._set_pw(10)

# Standard gain is ~50 == ~150mV (gain 1 == 3mV)
roboroach._set_gain(50)

# Standard number of pulses is 1
roboroach._set_np(1)
print("Initialized")

##########################
####### CALIBRATION
##########################

print("Starting calibration... (press <Ctrl-C> to finish)")
ref_pos = []
try:
	while True:
		pos = polaris_driver.getPositionFromBX("1801")
		if not 'miss' in pos:
			ref_pos.append(pos)

		time.sleep(0.1);
except KeyboardInterrupt:
	pass
ref_pos = np.mean(ref_pos[1:], axis=0)
print("\nPosition: ["+str(ref_pos[0])+", "+str(ref_pos[1])+", "+str(ref_pos[2])+"]")
print("Calibrated")

##########################
####### CONTROL
##########################

print("Line follower started! (press <Ctrl-C> to finish)")
POS_THRESH_POS = 1.5
POS_THRESH_NEG = -1.5
try:
	while True:
		pos = polaris_driver.getPositionFromBX("1801")
		if not 'miss' in pos:
			if pos[0] - ref_pos[0] > POS_THRESH_POS:
				roboroach._turn("right")
			elif pos[0] - ref_pos[0] < POS_THRESH_NEG:
				roboroach._turn("left")	

		time.sleep(0.1);
except KeyboardInterrupt:
	pass
print("\nEnd of control")

print("Finishing tracker...")
polaris_driver.stopTracking()
polaris_driver.close()
print("Tracker finished")