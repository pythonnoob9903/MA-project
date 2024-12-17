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

while get_rc_channel_value(6) == None: # checks if there is a rc_channel already connected, could be put in checks()
	log(f"currently no communication to radio: {get_rc_channel_value(6)}")
	time.sleep(1)
print(get_rc_channel_value)

while checks() is not True:
    checks()
    time.sleep(1)


Xcord = meters_to_coordinates(getcords()[0], getcords()[1],getcords()[2], vehicle)[0]
Ycord = meters_to_coordinates(getcords()[0], getcords()[1],getcords()[2], vehicle)[1]
#Zcord = getcords()[2]
Zcord = meters_to_coordinates(getcords()[0], getcords()[1],getcords()[2], vehicle)[2]
log(Xcord[0])
log(Ycord[0])
log(Zcord[0])
target = LocationGlobal(float(Xcord[0]), float(Ycord[0]), float(Zcord[0]))

log("geofence enaled")

while radiocontrol() is True:
    vehicle.mode = VehicleMode('GUIDED') #setting mode to guided
    while not vehicle.mode.name == "GUIDED":
        log(f"Not in GUIDED mode: {vehicle.mode.name}")
        time.sleep(1)
    log(f"in GUIDED mode: {vehicle.mode.name}")
    setup()

    log("starting vehicle.simple_takeoff")
    vehicle.simple_takeoff(target.alt)

    start_time = time.time()
    while True:
	    elapsed_time = time.time() - start_time
	    radiocontrol()
	    log("in takeoffloop")
	    current_altitude = vehicle.location.global_frame.alt
	    if int(current_altitude) >= int(target.alt) -0.1:
	    	log(f"target altitude reached{target.alt}")
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if int(current_altitude) >= 5 + int(current_altitude):
	    	log(f"drone is too far from home{current_altitude}") 
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if elapsed_time >= 10:
	    	log(f"Timoout, took to long to reach target altitude: {current_altitude}")
	    	return
	    time.sleep(1)
    
    vehicle.armed = False
    log(f"simple_takeoff did not work: mode {vehicle.mode.name}")

    setup()

    vehicle.simple_goto(target)
    while True:
	    elapsed_time = time.time() - start_time
	    radiocontrol()
	    log("in gotoloop")
	    current_altitude = vehicle.location.global_frame.alt
	    if int(current_altitude) >= int(target.alt) -0.1:
	    	log(f"target altitude reached{target.alt}")
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if int(current_altitude) >= 5 + int(current_altitude):
	    	log(f"drone is too far from home{current_altitude}") 
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if elapsed_time >= 10:
	    	log(f"Timoout, took to long to reach target altitude: {current_altitude}")
	    	return
	    time.sleep(1)
    
    vehicle.armed = False
    Vehicle.parameters["FENCE_ENABLE"] = 0
    log(f"geofence disabled")
    setup()

    log("starting vehicle.simple_takeoff")
    vehicle.simple_takeoff(target.alt)

    start_time = time.time()
    while True:
	    elapsed_time = time.time() - start_time
	    radiocontrol()
	    log("in takeoffloop")
	    current_altitude = vehicle.location.global_frame.alt
	    if int(current_altitude) >= int(target.alt) -0.1:
	    	log(f"target altitude reached{target.alt}")
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if int(current_altitude) >= 5 + int(current_altitude):
	    	log(f"drone is too far from home{current_altitude}") 
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if elapsed_time >= 10:
	    	log(f"Timoout, took to long to reach target altitude: {current_altitude}")
	    	return
	    time.sleep(1)
    
    vehicle.armed = False
    log(f"simple_takeoff did not work: mode {vehicle.mode.name}")

    setup()

    vehicle.simple_goto(target)
    while True:
	    elapsed_time = time.time() - start_time
	    radiocontrol()
	    log("in gotoloop")
	    current_altitude = vehicle.location.global_frame.alt
	    if int(current_altitude) >= int(target.alt) -0.1:
	    	log(f"target altitude reached{target.alt}")
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if int(current_altitude) >= 5 + int(current_altitude):
	    	log(f"drone is too far from home{current_altitude}") 
	    	vehicle.mode = VehicleMode["LOITER"]
	    	break
	    if elapsed_time >= 10:
	    	log(f"Timoout, took to long to reach target altitude: {current_altitude}")
	    	return
	    time.sleep(1)
    
    vehicle.armed = False
    setup()
    log("trying mavlink command")
    vehicle.mav.command_long_send(
    	vehicle.target_system,
    	vehicle.target_component,
    	mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    	0,  # Confirmation
    	0, 0, 0, 0,  # No specific yaw/lat/lon/alt params
    	0, 0, 1  # Target altitude (1)
	)

    time.sleep(5)

    vehicle.armed = False