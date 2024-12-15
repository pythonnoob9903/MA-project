from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
from coordinatesread import *
import math
from pymavlink import mavutil



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
		log('Giving back control to radio')
		return False
	else:
		log("control stays autonomous")
		return True

log(str(vehicle.battery.voltage) + " battery voltage")

initial_log()

while get_rc_channel_value(6) == None: # checks if there is a rc_channel already connected, could be put in checks()
	log(f"currently no communication to radio: {get_rc_channel_value(6)}")
	time.sleep(1)
print(get_rc_channel_value)

while radiocontrol() is True: 
	while checks(vehicle) is False:
		time.sleep(1)

	
	vehicle.mode = VehicleMode('GUIDED') #setting mode to guided
	while not vehicle.mode.name == "GUIDED":
			log(f"Not in GUIDED mode: {vehicle.mode.name}")
			time.sleep(1)
	log(f"in GUIDED mode: {vehicle.mode.name}")

	
	setup(vehicle) #arms and sets important parameters
	log(armed)
	time.sleep(5)
	Xcord = meters_to_coordinates(getcords()[0], getcords()[1],getcords()[2], vehicle)[0]
	Ycord = meters_to_coordinates(getcords()[0], getcords()[1],getcords()[2], vehicle)[1]
	Zcord = getcords()[2]
	#Zcord = meters_to_coordinates(getcords()[0], getcords()[1],getcords()[2], vehicle)[2]
	log(Xcord[0])
	log(Ycord[0])
	log(Zcord[0])


	time.sleep(5)
	if checks(vehicle) is False:
		log("Arming Checks failed midflight setting mode to loiter.")
		vehicle.mode = VehicleMode["LOITER"]
		break

	#target = LocationGlobal(Xcord[0], Ycord[0], Zcord[0]) # associates target to the target location
	target = LocationGlobalRelative(Xcord[0], Ycord[0], Zcord[0]) # associates target to the target location



	if radiocontrol() is False: 
		log("radiocontrol failed")
		vehicle.mode = VehicleMode["LOITER"]
		break

	# vehicle.simple_goto(target)
	vehicle.simple_takeoff(Zcord[0])

	log(f"{vehicle.mode.name}, --> mode")
	start_time = time.time() # starts timer for the the vehicle.simple_takeoff

	while True:
		elapsed_time = time.time() - start_time
		radiocontrol()
		log("in takeoffloop")
		current_altitude = vehicle.location.global_frame.alt
		if int(current_altitude) >= int(target.alt) -0.1:
			log(f"target altitude reached{target[2]}")
			break
		if int(current_altitude) >= 5 + int(current_altitude):
			log(f"drone is too far from home{current_altitude}") 
			vehicle.mode = VehicleMode["LOITER"]
			break
		if elapsed_time >= 5:
			log(f"Timoeout, took to long to reach target altitude: {current_altitude}")
			break
		time.sleep(1)
	
	vehicle.mode = VehicleMode('GUIDED')
	while not vehicle.mode.name == "GUIDED":
        	log(f"Not in Guided mode: {vehicle.mode.name}")
	log(f"in guided mode: {vehicle.mode.name}")
	
	radiocontrol()
	time.sleep(5)


	time.sleep(3)
	
	vehicle.mode = VehicleMode("RTL")
	while not vehicle.mode.name == "RTL":
		radiocontrol()
		log("Vehicle mode is not yet RTL")
		time.sleep(1) 
	time.sleep(1)
	break 
