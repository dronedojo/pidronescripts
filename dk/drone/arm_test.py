import time
import os
import platform
import sys

from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil
#############################
############DRONEKIT#################
vehicle = connect('udp:127.0.0.1:14550',wait_ready=True)
#########FUNCTIONS###########
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
	print("Look out! Virtual props are spinning!!")
        time.sleep(.5)

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
        arm()
        time.sleep(2)
        vehicle.mode = VehicleMode("LAND")
        while vehicle.mode!='LAND':
            time.sleep(1)
            print("Waiting for drone to land")
        time.sleep(1)
    except:
        pass
