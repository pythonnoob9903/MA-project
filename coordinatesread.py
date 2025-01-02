import pathlib
import math
import time
from datetime import datetime

def log(content): # logs what needs to be logged
    p = pathlib.Path(__file__).with_name('log.txt')
    o = p.open(mode="a")#opens the target file in append mode
    o.write("\n"+ str(datetime.now())+"	" +str(content))
    o.close()

def initial_log(): 
    current_time = datetime.now()
    log("\n")
    log(current_time)

def getcords(): # grabs coordinates out of coordinates.txt
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
        Z += [lines[i][counter2+1:]]
    return X, Y, Z


def meters_to_coordinates(X, Y, Z, vehicle): # converts coordinates in meters to coordinates in degrees with the global frame in connection to the home_location

    home_location = vehicle.home_location 
    log(f"home location: {home_location}")
    if not home_location:
        log("Waiting for home location to be set")
    while not home_location:
        time.sleep(1)

    Xcord = []
    Ycord = []
    Zcord = []

    for i in range(len(X)):
        earth_radius = 6378137.0
        
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
    tempbin = True
    if vehicle.is_armable is True:
        log("Vehicle is armable.") # does not check the Armingchecks
    else:
        log("Vehicle failed Arming_checks")
        tempbin = False
    if vehicle.battery.voltage <= 14:
        log(f'battery to low: {vehicle.battery.voltage}')
        tempbin = False
    if vehicle.gps_0.fix_type < 3:
        log(f"No 3D fix: {vehicle.gps_0.fix_type}")
        tempbin = False
    if vehicle.GPSinfo.epv >= 2:
        log(f"high GPS HDOP: {vehicle.GPSinfo.eph} (Vdop: {vehicle.GPSinfo.epv})")
        tempbin = False
    return tempbin

def safetyoptions_on_off(vehicle, on_off, VehicleMode): # 1 sets(and puts the drone into RTL) and 0 disables safety option--> if safetyoptions is enabled the script cannot control the drone
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

def distance_in_meters_to_target(vehicle, target): # calculates the current distance to the target distance
    earth_radius = 6378137.0
    current_location = vehicle.location.global_frame

    while current_location is None:
        log("Home location is not set.")
        time.sleep(1)

    # calculates horizontal distance to target destination
    Xdistance = math.radians(vehicle.location.global_frame.lon - target.lon) * earth_radius
    Ydistance = math.radians(vehicle.location.global_frame.lat - target.lat) * earth_radius * math.cos(math.radians(target.lat))
    
    #gets the vertical distance to the target location
    Zdistance = vehicle.location.global_frame.alt - target.alt

    distance = math.sqrt(Xdistance**2 + Ydistance**2 + Zdistance**2)
    log(f"distance to target: {distance}m, Hdop: {vehicle.GPSinfo.eph}, Vdop: {vehicle.GPSinfo.epv}")
    return distance