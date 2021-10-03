import time
import os
import platform
import sys

from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil
#############################
############DRONEKIT#################
vehicle = connect('udp:127.0.0.1:14550',wait_ready=True)
servo=14
high=1900
low=1100
#########FUNCTIONS###########
def controlServo(servo_number,pwm_value):
    msg = vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,
            pwm_value,
            0,
            0,
            0,
            0,
            0)
    vehicle.send_mavlink(msg)

###########Example execution#####
controlServo(servo,high)
time.sleep(1)
controlServo(servo,low)
#time.sleep(1)
