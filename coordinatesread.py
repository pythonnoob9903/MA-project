import pathlib
import math
import time
from datetime import datetime

def log(content):
    p = pathlib.Path(__file__).with_name('log.txt')
    o = p.open(mode="a")#opens the target file in append mode
    o.write("\n"+ str(datetime.now())+"	" +str(content))
    o.close()

def initial_log():
    current_time = datetime.now()
    log("\n")
    log(current_time)

def getcords():
    p = pathlib.Path(__file__).with_name('coordinates.txt')

    X = []
    Y = []
    Z = []

    coordinates = p.open('r')
    lines = coordinates.readlines()
    coordinates.close()

    for i in range(len(lines)):
        counter1 = 0
        while lines[i][counter1] != ";":
            counter1 += 1
        X += [lines[i][0:counter1]]
        counter2 = counter1 + 1
        while lines[i][counter2] != ";":
            counter2 += 1
        Y += [lines[i][counter1+1:counter2]]
        Z += [lines[i][counter2+1:-1]]
    return X, Y, Z


def meters_to_coordinates(X, Y, Z, vehicle): # converts coordinates in meters to coordinates in degrees with the global frame in connection to the home_location

    home_location = vehicle.home_location 
    log(f"home location: {home_location}")
    if not home_location:
        log("Waiting for home location to be set")
    while not home_location:
        time.sleep(1)

    global Xcord    # makes the coordinates global for no further redefinition
    global Ycord
    global Zcord
    Xcord = []
    Ycord = []
    Zcord = []

    for i in range(len(X)):
        earth_radius = 6378137.0
        #changeX = math.degrees(float(X[i]) / earth_radius) # changes the Xcord to coordinates in the global frame and adds them to the coordinates from the home location
        #changeY = math.degrees(float(Y[i]) / (earth_radius)) #* math.cos(math.radians(home_location.lat))))
        #changeZ = float(Z[i])
        
        changeX = home_location.lat + math.degrees(float(X[i]) / earth_radius) # changes the Xcord to coordinates in the global frame and adds them to the coordinates from the home location
        changeY = home_location.lon + math.degrees(float(Y[i]) / (earth_radius * math.cos(math.radians(home_location.lat))))
        changeZ = home_location.alt + float(Z[i])

        Xcord += [changeX]
        Ycord += [changeY]
        Zcord += [changeZ]
    
    log(f"Xcoordinates {Xcord}")
    log(f"Ycoordinates {Ycord}")
    log(f"Zcoordinates {Zcord}")

    return Xcord, Ycord, Zcord



def setup(vehicle, VehicleMode): #sets up the initial variables for flight and calculates the target coordinates
    log("Speed_Up and groundspeed set/vehicle armed.")
    vehicle.groundspeed = 1
    vehicle.parameters["PILOT_SPEED_UP"] = 100
    safetyoptions_on_off(vehicle, 0, VehicleMode)
    vehicle.armed = True

def checks(vehicle): #checks arming checks and can stall the while loop if it fails.
    vehicle.parameters["ARMING_CHECK"] = 1 # enables Arming_checks.
    tempbin = False
    if vehicle.is_armable is True:
        log("Vehicle is armable.")
        tempbin = True
    else:
        log("Vehicle failed Arming_checks")
    if vehicle.battery.voltage <= 14:
        log(f'battery to low: {vehicle.battery.voltage}')
        tempbin = False
    if vehicle.gps_0.fix_type < 3:
        log(f"No 3D fix: {vehicle.gps_0.fix_type}")
    return tempbin

def safetyoptions_on_off(vehicle, on_off, VehicleMode): # 1 sets(and puts the drone into RTL) and 0 disables safety option
    if on_off == 1:
        vehicle.mode = VehicleMode('RTL')
        while not vehicle.mode.name == "RTL":
        	log(f"Not in RTL mode: {vehicle.mode.name}")
        log(f"in RTL mode: {vehicle.mode.name}")
        vehicle.parameters["BRD_SAFETYOPTION"] = 3
        log("safetyoption bitmask set to three")
        vehicle.parameters["BRD_SAFETY_DEFLT"] = 1
        log("safety switch enabled")
        
    else: 
        vehicle.parameters["BRD_SAFETYOPTION"] = 0
        log("safetyoption bitmask set to zero")
        vehicle.parameters["BRD_SAFETY_DEFLT"] = 0
        log("safety switch disabled")

def flytoallcoordinates(vehicle, VehicleMode):
    #vehicle.mode = VehicleMode["GUIDED"]
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
            log(f"in takeoffloop-> current altitude: {current_altitude}")
            if current_altitude >= target.alt *0.95 and current_latitude >= Xcord * 0.95 and current_longitude >= Zcord * 0.95:
            	log(f"target {i} reached {vehicle.location.global_frame}")
            	break
            if current_altitude >= 5 + current_altitude or current_latitude >= Xcord + 5 or current_longitude >= Zcord + 5: # does not work with coordinates that go into the negative direction of the starting point
            	log(f"drone is too far from home :{vehicle.location.global_frame}")
            	vehicle.mode = VehicleMode["RTL"]
            	break
            if elapsed_time >= 10:
            	log(f"Timeout, took to long to reach target altitude: {vehicle.location.global_frame}")
            	safetyoptions_on_off(vehicle, 1, VehicleMode)
            	vehicle.mode = VehicleMode["RTL"]
            	while vehicle.mode.name != "RTL":
            		log(f"vehiclemode not RTL: {vehicle.mode.name}")
            	log(f"changing to RTL: {vehicle.mode.name}")
            	break
            time.sleep(1)