import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

import asyncio

# vehicle = connect('udpout:192.168.1.16:14552')
# vehicle = connect('COM7')
vehicle = connect('tcp:127.0.0.1:5763')
vehicle.mode    = VehicleMode("STABILIZE")
is_arm = 0
mav_alt = 0
need_alt = 20
need_heading = 100
vz = 1500
yaw = 1500
pitch = 1500
right_direction = False
right_height = False

print('+++++++++++++++++')
print('ARM !!!')
print('+++++++++++++++++')
if (is_arm == 0):
    msg = vehicle.message_factory.command_long_encode(
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #command
    0, #confirmation
    1, # param 1
    0, # param 2
    0, # param 3
    0, # param 4
    0, 0, 0)    # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)
    is_arm = 1
    time.sleep(3)

if (is_arm == 1):
    while True:
        try:
            mav_alt = vehicle.location.global_relative_frame.alt
        except:
            pass
        if (mav_alt is None): mav_alt = 0
        print('+++++++++++++++++++++++++')
        print(mav_alt)
        print('======= GO UP !!! =======')
        print('+++++++++++++++++++++++++')

        d_alt = mav_alt - need_alt

        print("Pitch:" + str(vehicle.attitude.pitch))

        if not right_height or not right_direction:
            if (d_alt < -10):
                # Need UP
                vz = 1500
            elif (-10 < d_alt < 0):
                # Need UP
                vz = 1480
            elif (d_alt > 0):
                # Need Down
                vz = 1478
                right_height = True
            if vehicle.heading and vehicle.heading >= need_heading + 30:
                yaw = 1450
            elif vehicle.heading and vehicle.heading <= need_heading - 30:
                yaw = 1550
            elif vehicle.heading and vehicle.heading >= need_heading + 1:
                yaw = 1479
            elif vehicle.heading and vehicle.heading <= need_heading - 1:
                yaw = 1521
            elif vehicle.heading and vehicle.heading == need_heading:
                yaw = 1500
                right_direction = True

        elif right_height and right_direction:
            pitch = 1000
            if d_alt < 0:
                vz = 1473
            elif d_alt > 0:
                vz = 1472
        vehicle.channels.overrides = {'2': pitch, '3': vz, '4': yaw,}
        time.sleep(0.8)