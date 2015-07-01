from control_api import RoboRoach, time
from control_api import PolarisDriver
from select import select as sel
import numpy as np
import math
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

print("Starting path acquisition... (press <Ctrl-C> to finish)")
ref_pos = np.array([])
try:
	while True:
		pos = polaris_driver.getPositionFromBX("1801")
		if not 'miss' in pos:
			ref_pos = np.vstack([ref_pos, pos])

		time.sleep(0.1);
except KeyboardInterrupt:
	pass
print("Path acquired!")

##########################
####### CONTROL
##########################

print("Path follower started! (press <Ctrl-C> to finish)")
pos_vec = np.array([0, 0, 0])
i = 0
try:
	while i < len(pos)-1
		ref_vec = [pos[i], pos[i+1]]

		curr_pos = polaris_driver.getPositionFromBX("1801")
		if not 'miss' in curr_pos:
			updatepos(pos_vec, curr_pos)
			dot = [dotproduct(pos_vec[:,1],ref_vec[:,1], dotproduct(pos_vec[:,2],ref_vec[:,2]), dotproduct(pos_vec[:,3],ref_vec[:,3])]

			if dot > 1:
				roboroach._turn("right")
			elif dot < -1:
				roboroach ._turn("left")

		if dist(pos[i], curr_pos) > dist(pos[i+1], curr_pos)
			i = i + 1;

		time.sleep(0.1)
except KeyboardInterrupt:
	pass
print("\nEnd of control")

print("Finishing tracker...")
polaris_driver.stopTracking()
polaris_driver.close()
print("Tracker finished")

##########################
####### AUXILIAR FUNCTIONS
##########################
def dist(v1, v2):
	return np.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2 + (v1[2] - v2[2])**2)

def updatepos(v, value):
	v[0] = v[1]
	v[1] = value
	return v

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))