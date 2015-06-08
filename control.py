from control_api import RoboRoach

roboroach = RoboRoach(mac_address="90:59:AF:14:08:E8")

roboroach._turn("right");

# Create reference
# pos = []

# Get data from Polaris
# a = getDataFromBX()

# Apply control
# if a[z] > pos[z]: # a > b == a - b > THRESHOLD
#   goToRight()
# if a[z] < pos[z]:
  # goToLeft()