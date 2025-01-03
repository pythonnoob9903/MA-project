from dronekit import connect, VehicleMode, LocationGlobal
import time
from coordinatesread import *
import math
from pymavlink import mavutil

initial_log()

vehicle = connect('/dev/serial0', baud=912600, wait_ready=True)
log("connected to Fc")

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
		safetyoptions_on_off(vehicle, 1, VehicleMode)
		vehicle.mode = VehicleMode("RTL")
		while vehicle.mode.name != "RTL":
			log(f"vehiclemode not set to RTL: {vehicle.mode.name}")
			time.sleep(0.5)
		log(f"vehiclemode set to RTL: {vehicle.mode.name}")
		return False
	else:
		log("control stays autonomous")
		return True


@vehicle.on_message("STATUSTEXT") 
def prearm_failures(vehicle, name, message): # is called when a message appears as statustext and saves it to the log.txt file
	if "Prearm" in message.text:
		log(f"Prearm failure: {message.text}")
	elif "Arm" in message.text:
		log(f"Arm problem: {message.text}")
	else:
		log(f"unexpected Statustext in format: {message}")


def flytoallcoordinates(vehicle, VehicleMode): # loops through all coordinates provided by coordinates.txt
	vehicle.mode = VehicleMode("GUIDED")
	radiocontrol()

	while vehicle.mode.name != "GUIDED" and radiocontrol() is True:
		log(f"vehiclemode not GUIDED: {vehicle.mode.name}")
		radiocontrol()
	log(f"vehiclemode GUIDED: {vehicle.mode.name}")

	for i in range(len(Xcord)):
		target = LocationGlobal(Xcord[i], Ycord[i], Zcord[i])
		vehicle.simple_goto(target)
		radiocontrol()
		log(f"forloop number: {i+1}")
		log(f"target location: {target}")
		log(f"vehicle mode: {vehicle.mode.name}")
        
		start_time = time.time()

		while True: # breakoff, if the target is not reached in 10 seconds or the drone flies more than 5 meters away
            
			elapsed_time = time.time() - start_time
			radiocontrol()

			current_altitude = vehicle.location.global_frame.alt
			current_longitude = vehicle.location.global_frame.lon
			current_latitude = vehicle.location.global_frame.lat
			current_location = vehicle.location.global_frame


			log(f"in gotoloop-> current altitude: {vehicle.location.global_frame}")
			if distance_in_meters_to_target(vehicle, target) <= 0.1:
				log(f"target {i} reached {vehicle.location.global_frame}")
				break
			if distance_in_meters_to_target(vehicle, target) >= 5:
				log(f"drone is too far from home:{vehicle.location.global_frame}")
				vehicle.mode = VehicleMode("RTL")
				break
			if elapsed_time >= 10:
				log(f"target not reached, going to next target: initial target{target}, current position {current_location}, distance: {distance_in_meters_to_target(vehicle, target)}")
				break
			time.sleep(1)





log(str(vehicle.battery.voltage) + " battery voltage")
log(f"vehicle mode: {vehicle.mode.name}")


while get_rc_channel_value(6) == None: # checks if there is a rc_channel already connected, might be put in later on checks()
	log(f"currently no communication to radio: {get_rc_channel_value(6)}")
	time.sleep(1)
log(f"rc channel value:{get_rc_channel_value(6)}")

vehicle.mode = VehicleMode('GUIDED')
while not vehicle.mode.name == "GUIDED":
		log(f"Not in GUIDED mode: {vehicle.mode.name}")
		time.sleep(1)
log(f"in GUIDED mode: {vehicle.mode.name}")


setup(vehicle, VehicleMode) #arms and sets important parameters
vehicle.armed = True
time.sleep(1)
vehicle.armed = False


while radiocontrol() is True: 
	while checks(vehicle) is False:
		time.sleep(1)
		log(f"checks {checks(vehicle)}")
	
	vehicle.mode = VehicleMode('GUIDED') #setting mode to guided
	while not vehicle.mode.name == "GUIDED":
			log(f"Not in GUIDED mode: {vehicle.mode.name}")
			time.sleep(1)
	log(f"in GUIDED mode: {vehicle.mode.name}")


	log(f"Hdop: {vehicle.gps_0.eph}, Vdop: {vehicle.gps_0.epv}, sattelites visible: {vehicle.gps_0.satellites_visible}")

	log(f"current position: {vehicle.location.global_frame}")

	log(f"armed, {vehicle.armed}")
	print("vehicle armed")
	Xcord, Ycord, Zcord = meters_to_coordinates(getcords()[0], getcords()[1], getcords()[2], vehicle) # grabs variables Xcord, Ycord and Zcord
	time.sleep(5)

	target = LocationGlobal(Zcord[0], Ycord[0], Zcord[0])
	log(f"target location: {target}")


	if checks(vehicle) is False:
		safetyoptions_on_off(vehicle, 1, VehicleMode)
		log("Arming Checks failed midflight setting mode to RTL.")
		vehicle.mode = VehicleMode("RTL")
		break

	radiocontrol()
	
	vehicle.armed = True
	log(f"vehicle armed: {vehicle.armed}")
	print("vehicle armed")
	time.sleep(10)
	
	log(f"board safety option bitmask set to: {vehicle.parameters['BRD_SAFETYOPTION']}")
	# vehicle.simple_goto(target)
	vehicle.simple_takeoff(Zcord[0])

	log(f"{vehicle.mode.name}, --> mode")
	start_time = time.time() # starts timer for the the vehicle.simple_takeoff
	radiocontrol()

	while True: # is killing the simple takeoff if too much time has gone by or it deviated to far from the origin
		elapsed_time = time.time() - start_time
		radiocontrol()
		current_altitude = vehicle.location.global_frame.alt
		log(f"in takeoffloop-> current altitude: {current_altitude}")
		if target.alt - current_altitude <=  0.1:
			log(f"target altitude reached {target.alt}, {current_altitude}")
			break
		if target.alt - current_altitude >= 5:
			log(f"drone is too far from home :{current_altitude}") 
			vehicle.mode = VehicleMode("RTL")
			break
		if elapsed_time >= 10:
			log(f"Timeout, took to long to reach target altitude: {current_altitude}, target altitude: {target.alt}")
			safetyoptions_on_off(vehicle, 1, VehicleMode)
			vehicle.mode = VehicleMode("RTL")
			while vehicle.mode.name != "RTL":
				log(f"vehiclemode not RTL: {vehicle.mode.name}")
			log(f"changing to RTL: {vehicle.mode.name}")
			break
		time.sleep(1)

	
	flytoallcoordinates(vehicle, VehicleMode)
	

	log(f"location before RTL: {vehicle.location.global_frame}")

	vehicle.mode = VehicleMode('RTL')
	while not vehicle.mode.name == "RTL":
        	log(f"Not in RTL mode: {vehicle.mode.name}")
	log(f"in RTL mode: {vehicle.mode.name}")
	safetyoptions_on_off(vehicle, 1, VehicleMode)
	break
