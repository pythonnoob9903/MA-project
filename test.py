#from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
from coordinatesread import *
import math
#from pymavlink import mavutil

Xcord, Ycord, Zcord = meters_to_coordinates(getcords()[0], getcords()[1], getcords()[2])
#def meters_to_coordinates(X, Y, Z):#, vehicle): # converts coordinates in meters to coordinates in degrees with the global frame in connection to the home_location
#
#    #home_location = vehicle.home_location 
#    #log(f"home location: {home_location}")
#    #if not home_location:
#    #    log("Waiting for home location to be set")
#    #while not home_location:
#    #    time.sleep(1)
#
#    global Xcord    # makes the coordinates global for no further redefinition
#    global Ycord
#    global Zcord
#    Xcord = []
#    Ycord = []
#    Zcord = []
#
#    for i in range(len(X)):
#        earth_radius = 6378137.0
#        changeX = math.degrees(float(X[i]) / earth_radius) # changes the Xcord to coordinates in the global frame and adds them to the coordinates from the home location
#        changeY = math.degrees(float(Y[i]) / (earth_radius)) #* math.cos(math.radians(home_location.lat))))
#        changeZ = float(Z[i])
#        
#        #changeX = home_location.lat + math.degrees(float(X[i]) / earth_radius) # changes the Xcord to coordinates in the global frame and adds them to the coordinates from the home location
#        #changeY = home_location.lon + math.degrees(float(Y[i]) / (earth_radius * math.cos(math.radians(home_location.lat))))
#        #changeZ = home_location.alt + float(Z[i])
#
#        Xcord += [changeX]
#        Ycord += [changeY]
#        Zcord += [changeZ]
#    
#    log(f"Xcoordinates {Xcord}")
#    log(f"Ycoordinates {Ycord}")
#    log(f"Zcoordinates {Zcord}")
#
#    return Xcord, Ycord, Zcord


print(Xcord)
print(Zcord)
print(Ycord)
target = [Xcord[0], Ycord[0], Zcord[0]]

print(target)