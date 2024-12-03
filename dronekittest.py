from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from coordinatesread import *
import math

vehicle = connect('/dev/serial0', baud=912600, wait_ready=True)



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
		print("control stays autonomous")
		return True

print(vehicle.battery)
vehicle.parameters['ARMING_CHECK'] = 1
while radiocontrol(): 
	while checks() is False:
		time.sleep(1)

	setup()
	Xcord = meters_to_coordinates(getcords()[0], getcords()[1], vehicle)[0]
	Ycord = meters_to_coordinates(getcords()[0], getcords()[1], vehicle)[1]
	Zcord = getcords()[2]

	radiocontrol()

	if checks() is False:
		print("Arming Checks failed midflight setting mode to loiter.")
		vehicle.mode = VehicleMode["LOITER"]
		break

	target = LocationGlobalRelative(Xcord[0], Ycord[0], Zcord[0])
	vehicle.mode = VehicleMode("GUIDED") # sets the vehicle mode to guided to be used with the vehicle.simple_goto
	
	vehicle.simple_takeoff(Zcord)
	while True:
		if vehicle.location.global_relative_frame.alt >= Zcord -0.1:
			print("Heigth reached")
			break
		time.sleep(1)
        	
	radiocontrol()

	vehicle.simple_goto(target)

	time.sleep(3)
	
	vehicle.mode = VehicleMode("LOITER")
	while not vehicle.mode.name == "LOITER":
		radiocontrol()
		print("Vehicle mode is not yet Loiter")
		time.sleep(1) 
	time.sleep(1)
	break 
	# control interrupt(radiocontrol())
	
	# look up the direction the drone is facing
	# get difference in current GPS and target
	# check interrupt
	# adjust height
	# set mode to loiter --> just to keep the height(probably not loiter)
	# fly in direction for a short period and try last step again
	# if target distance achieved--> check if it is the last target if no: go to next target else repeat
	# if last target: interrupt
