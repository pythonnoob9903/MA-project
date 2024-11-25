from dronekit import connect, VehicleMode
import time
from coordinatesreadtest import *

Xcord = getcords()[0]
Ycord = getcords()[1]
Zcord = getcords()[2]

vehicle = connect('/dev/serial0', baud=912600, wait_ready=True)

latest_rc_channels = None

@vehicle.on_message("RC_CHANNELS")
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

while radiocontrol() is True: 
	# set home point with intitial GPS data--> is already set when arming the drone
	# read a file to set a target array(use a flat file with only ascii informations probably .txt)
	# set target from the array(list or dict)
	vehicle.armed = True
	# read the sensors/GPS
	radiocontrol()
	# control interrupt(radiocontrol())
	
	# look up the direction the drone is facing
	# get difference in current GPS and target
	# check interrupt
	# adjust height
	# set mode to loiter --> just to keep the height(probably not loiter)
	# fly in direction for a short period and try last step again
	# if target distance achieved--> check if it is the last target if no: go to next target else repeat
	# if last target: interrupt
