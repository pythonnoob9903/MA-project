#from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
from coordinatesread import *
import math
#from pymavlink import mavutil

def coordinates_to_meters_from_starting_point(vehicle, target):
    earth_radius = 6378137.0
    current_location = vehicle.global_frame

    while home_location is None:
        log("Home location is not set.")
        time.sleep(1)

    # calculates horizontal distance to target destination
    Xdistance = math.radians(vehicle.global_frame.lon - target.lon) * earth_radius
    Ydistance = math.radians(vehicle.global_frame.lat - target.lat) * earth_radius * math.cos(math.radians(home_location))
    
    #gets the vertical distance to the target location
    Zdistance = vehicle.global_frame.alt - target.alt

    distance = math.sqrt(Xdistance**2 + Ydistance**2 + Zdistance**2)

    return distance