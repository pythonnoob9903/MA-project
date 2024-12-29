from dronekit import connect, VehicleMode, LocationGlobal
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


def flytoallcoordinates(vehicle, VehicleMode):
    vehicle.mode = VehicleMode("GUIDED")
    radiocontrol()

    while vehicle.mode.name != "GUIDED":
        log(f"vehiclemode not GUIDED: {vehicle.mode.name}")
        radiocontrol()
    log(f"vehiclemode GUIDED: {vehicle.mode.name}")

    for i in range(len(Xcord)):
        target = LocationGlobal(Xcord[i], Ycord[i], Zcord[i])
        vehicle.simple_goto(target)
        radiocontrol()

        start_time = time.time()
        while True: # timeout if the target is not reached in 10 seconds or the drone flies more than 5 meters away
            elapsed_time = time.time() - start_time
            radiocontrol()
            current_altitude = vehicle.location.global_frame.alt
            current_longitude = vehicle.location.global_frame.lon
            current_latitude = vehicle.location.global_frame.lat
            log(f"in gotoloop-> current altitude: {vehicle.location.global_frame}")
            if current_altitude >= target.alt - 0.1 and current_latitude >= float(Xcord[i]) * 0.95 and current_longitude >= float(Zcord[i]) * 0.95:
            	log(f"target {i} reached {vehicle.location.global_frame}")
            	break
            if current_altitude >= 5 + current_altitude or current_latitude >= Xcord[i] + 1 or current_longitude >= Zcord[i] + 1: # does not work with coordinates that go into the negative direction of the starting point/somehow need to get this to a distance 5 m
            	log(f"drone is too far from home:{vehicle.location.global_frame}")
            	vehicle.mode = VehicleMode("RTL")
            	break
            if elapsed_time >= 10:
            	log(f"Timeout, took to long to reach target altitude: {vehicle.location.global_frame}")
            	safetyoptions_on_off(vehicle, 1, VehicleMode)
            	vehicle.mode = VehicleMode("RTL")
            	while vehicle.mode.name != "RTL":
            		log(f"vehiclemode not RTL: {vehicle.mode.name}")
            	log(f"changing to RTL: {vehicle.mode.name}")
            	break
            time.sleep(1)

log(str(vehicle.battery.voltage) + " battery voltage")
log(vehicle.mode.name)
initial_log()

while get_rc_channel_value(6) == None: # checks if there is a rc_channel already connected, could be put in checks()
	log(f"currently no communication to radio: {get_rc_channel_value(6)}")
	time.sleep(1)
log(get_rc_channel_value)

setup(vehicle, VehicleMode)
vehicle.armed = True
time.sleep(1)
vehicle.armed = False


while radiocontrol() is True: 
	while checks(vehicle) is False:
		time.sleep(1)

	
	vehicle.mode = VehicleMode('GUIDED') #setting mode to guided
	while not vehicle.mode.name == "GUIDED":
			log(f"Not in GUIDED mode: {vehicle.mode.name}")
			time.sleep(1)
	log(f"in GUIDED mode: {vehicle.mode.name}")

	
	setup(vehicle, VehicleMode) #arms and sets important parameters
	log(f"armed, {vehicle.armed}")
	print("vehicle armed")
	Xcord, Ycord, Zcord = meters_to_coordinates(getcords()[0], getcords()[1], getcords()[2], vehicle) # gets the global variables Xcord, Ycord and Zcord
	time.sleep(5)

	target = LocationGlobal(Xcord[0], Ycord[0], Zcord[0]) # associates target to the target location
	#target = LocationGlobalRelative(Xcord[0], Ycord[0], Zcord[0]) # associates target to the target location
	target1 = LocationGlobal(Xcord[1], Ycord[1], Zcord[1])

	time.sleep(5)
	if checks(vehicle) is False:
		log("Arming Checks failed midflight setting mode to RTL.")
		vehicle.mode = VehicleMode("RTL")
		break

	radiocontrol()
	
	vehicle.armed = True
	log(f"vehicle armed: {vehicle.armed}")
	print("vehicle armed")

	log(f"board safety option bitmask set to: {vehicle.parameters['BRD_SAFETYOPTION']}")
	# vehicle.simple_goto(target)
	vehicle.simple_takeoff(Zcord[0])

	log(f"{vehicle.mode.name}, --> mode")
	start_time = time.time() # starts timer for the the vehicle.simple_takeoff
	time.sleep(2)

	while True: # is killing the simple takeoff if too much time has gone by or it deviated to far from the origin
		elapsed_time = time.time() - start_time
		radiocontrol()
		current_altitude = vehicle.location.global_frame.alt
		log(f"in takeoffloop-> current altitude: {current_altitude}")
		if float(current_altitude) >= float(target.alt) - 0.1:
			log(f"target altitude reached {target.alt}, {current_altitude}")
			break
		if int(current_altitude) >= 5 + int(current_altitude):
			log(f"drone is too far from home :{current_altitude}") 
			vehicle.mode = VehicleMode("LOITER")
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
	safetyoptions_on_off(vehicle, 0, VehicleMode)
	break
