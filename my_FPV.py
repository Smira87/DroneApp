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
    time.sleep(1)

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

        if not right_height and not right_direction:
            if (d_alt < -10):
                # Need UP
                vz = 1500
            if (-10 < d_alt < 0):
                # Need UP
                vz = 1480
            if (d_alt > 0):
                # Need Down

                vz = 1478

        vehicle.channels.overrides = {'3': vz}
        time.sleep(0.8)