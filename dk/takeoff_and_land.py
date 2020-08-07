import time
import os
import platform
import sys

from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil
#############################

targetAltitude=0
if len(sys.argv)>1:
    targetAltitude=float(sys.argv[1])
else:
    targetAltitude=1

############DRONEKIT#################
vehicle = connect('udp:127.0.0.1:14550',wait_ready=True)
vehicle.parameters['PLND_ENABLED']=0
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30
#########FUNCTIONS###########
def arm_and_takeoff(targetHeight):
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
	print("Look out! Virtual props are spinning!!")
	vehicle.simple_takeoff(targetHeight) ##meters

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")

	return None

########CONTROL DRONE
def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

if __name__=='__main__':
    try:
        arm_and_takeoff(targetAltitude)
        vehicle.mode = VehicleMode("LAND")
        while vehicle.mode!='LAND':
            time.sleep(1)
            print("Waiting for drone to land")
        time.sleep(1)
    except:
        pass
cv2.destroyAllWindows()
