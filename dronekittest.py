from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from coordinatesread import *
import math

vehicle = connect('/dev/serial0', baud=912600, wait_ready=True)


Xcord = meters_to_coordinates(getcords()[0], getcords()[1], vehicle)[0]
Ycord = meters_to_coordinates(getcords()[0], getcords()[1], vehicle)[1]
Zcord = getcords()[2]


latest_rc_channels = None

@vehicle.on_message("RC_CHANNELS") # listens to the RC_Channels
def rc_channel_listener(vehicle, name, message): 
	global latest_rc_channels
	latest_rc_channels = message

def get_rc_channel_value(channel_number): # checks the value of a certain channel
	global latest_rc_channels
	if latest_rc_channels is None:
		return None
	channel_value = getattr(latest_rc_channels, f"chan{channel_number}_raw", None)
	return channel_value

def radiocontrol(): # checks if the switch is still on the right position to fly autonomously
	if int(get_rc_channel_value(6)) >= 1800:
		print('Giving back control to radio')
		return False
	else:
		return True
print(vehicle.battery)
vehicle.parameters['ARMING_CHECK'] = 1
while radiocontrol() is True: 
	while checks() is False:
		time.sleep(1)
	# set home point with intitial GPS data--> is already set when arming the drone
	# read a file to set a target array(use a flat file with only ascii informations probably .txt) --> done by coordinatesread.py
	# set target from the array(list or dict)
	setup()
	radiocontrol()
	if checks() is False:
		break
	target = LocationGlobalRelative(Xcord[0], Ycord[0], Zcord[0])
	# read the sensors/GPS
	vehicle.mode = VehicleMode("GUIDED") # sets the vehicle mode to guided to be used with the vehicle.simple_goto
	vehicle.simple_takeoff(Zcord)
	radiocontrol()
	if checks() is False:
		break
	vehicle.simple_goto(target)
	vehicle.mode = VehicleMode("LOITER")
	time.sleep(1)
	# control interrupt(radiocontrol())
	
	# look up the direction the drone is facing
	# get difference in current GPS and target
	# check interrupt
	# adjust height
	# set mode to loiter --> just to keep the height(probably not loiter)
	# fly in direction for a short period and try last step again
	# if target distance achieved--> check if it is the last target if no: go to next target else repeat
	# if last target: interrupt
