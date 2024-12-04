from pathlib import Path
import math
import time


def getcords():
    p = Path(__file__).with_name('coordinates.txt')

    Xcord = []
    Ycord = []
    Zcord = []

    coordinates = p.open('r')
    lines = coordinates.readlines()
    coordinates.close()

    for i in range(len(lines)):
        counter1 = 0
        while lines[i][counter1] != ";":
            counter1 += 1
        Xcord += [lines[i][0:counter1]]
        counter2 = counter1 + 1
        while lines[i][counter2] != ";":
            counter2 += 1
        Ycord += [lines[i][counter1+1:counter2]]
        Zcord += [lines[i][counter2+1:-1]]
    return Xcord, Ycord, Zcord


def meters_to_coordinates(Xcord, Ycord, vehicle): # converts coordinates in meters to coordinates in degrees with the global frame in connection to the home_location


    home_location = vehicle.home_location 
    if not home_location:
        print("Waiting for home location to be set")
    while not home_location:
        time.sleep()

    newXcord = []
    newYcord = []
    for i in range(len(Xcord)):
        earth_radius = 6378137.0
        changeX = home_location.lat + math.degrees(int(Xcord[i]) / earth_radius) # changes the Xcord to coordinates in the global frame and adds them to the coordinates from the home location
        changeY = home_location.lon + math.degrees(int(Ycord[i]) / (earth_radius * math.cos(math.radians(home_location.lat))))
        newXcord += [changeX]
        newYcord += [changeY]
    
    return newXcord, newYcord

def setup(vehicle): #sets up the initial variables for flight
    print("Initial mode and groundspeed set/vehicle armed.")
    vehicle.groundspeed = 1
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Not in Guided mode")
    vehicle.parameters["PILOT_SPEED_UP"] = 50
    vehicle.armed = True

def checks(vehicle): #checks arming checks and can stall the while loop if it fails.
    vehicle.parameters["ARMING_CHECK"] = 1 # enables Arming_checks.
    tempbin = False
    if vehicle.is_armable is True:
        print("Vehicle is armable.")
        tempbin = True
    else:
        print("Vehicle failed Arming_checks")
    if vehicle.battery.voltage <= 14:
        print(f'battery to low: {vehicle.battery.voltage}')
        tempbin = False
    return tempbin
