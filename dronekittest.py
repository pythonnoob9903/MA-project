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

while get_rc_channel_value(6) == None: # checks if there is a rc_channel already connected, could be put in checks()
	print(f"currently no communication to radio: {get_rc_channel_value(6)}")
	time.sleep(1)
print(get_rc_channel_value(6))

while radiocontrol() is True: 
	while checks(vehicle) is False:
		time.sleep(1)

	setup(vehicle)
	time.sleep(5)
	Xcord = meters_to_coordinates(getcords()[0], getcords()[1], vehicle)[0]
	Ycord = meters_to_coordinates(getcords()[0], getcords()[1], vehicle)[1]
	Zcord = getcords()[2]
	print(Xcord[0])
	print(Ycord[0])
	print(Zcord[0])


	radiocontrol()
	time.sleep(5)
	if checks(vehicle) is False:
		print("Arming Checks failed midflight setting mode to loiter.")
		vehicle.mode = VehicleMode["LOITER"]
		break

	target = LocationGlobalRelative(Xcord[0], Ycord[0], Zcord[0])
	
	vehicle.simple_takeoff(Zcord)
	while True:
		if vehicle.location.global_relative_frame.alt >= Zcord -0.1:
			radiocontrol()
			print("Heigth reached")
			break
		time.sleep(1)
    
	radiocontrol()
	time.sleep(5)
	vehicle.simple_goto(target)

	time.sleep(3)
	
	vehicle.mode = VehicleMode("LOITER")
	while not vehicle.mode.name == "LOITER":
		radiocontrol()
		print("Vehicle mode is not yet Loiter")
		time.sleep(1) 
	time.sleep(1)
	break 