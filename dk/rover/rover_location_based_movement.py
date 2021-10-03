##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse


#########FUNCTIONS#################

#connection_string='/dev/ttyTHS1'
#baud=57600

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect


	vehicle = connect(connection_string,baud=57600,wait_ready=True)

	return vehicle

def arm():
	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Vehicle is now armed.")

	return None

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon
	
	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.05:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None
##########MAIN EXECUTABLE###########

wp1 = LocationGlobalRelative(36.00550,-95.86124,10)
wp2 = LocationGlobalRelative(36.00607,-95.86107,10)
wp3 = LocationGlobalRelative(36.00604,-95.86037,10)

vehicle = connectMyCopter()

vehicle.parameters['WP_SPEED']=2

arm()


goto(wp1)
goto(wp2)
goto(wp3)

vehicle.mode = VehicleMode("RTL")


while vehicle.mode!='RTL':
	print("Waiting for drone to enter RTL flight mode")
	time.sleep(1)
print("Vehicle now in RTL mode. Driving home.")
