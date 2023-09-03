import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

import asyncio

# vehicle = connect('udpout:192.168.1.16:14552')
# vehicle = connect('COM7')
def up_turn_go(needed_alt = 15, needed_heading = 0, needed_distance = 35):
    need_alt = needed_alt
    need_heading = needed_heading
    #need_distance = needed_distance
    if need_alt < 15: need_alt = 15
    if needed_distance < 35: needed_distance = 35
    vehicle = connect('tcp:127.0.0.1:5763')
    vehicle.mode    = VehicleMode("STABILIZE")
    is_arm = 0
    mav_alt = 0
    vz = 1500
    yaw = 1500
    pitch = 1500
    right_direction = False
    right_height = False
    start_heading_home = False
    heading_home = False
    distance = 0
    timeframe = 0.8
    start_forward = False
    start_backward = False
    fly_forward_time = 0

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

            print("Distance : " + str(distance))
            print("Fly forward time : " + str(fly_forward_time))


            if not right_height and not right_direction:
                if (d_alt < -12):
                    # Need UP
                    vz = 1500
                elif (-5 < d_alt < 0):
                    # Need UP
                    vz = 1480
                elif (d_alt > 0):
                    # Need Down
                    vz = 1478
                    right_height = True
            elif right_height and not right_direction:
                pitch = 1500
                if (-12 < d_alt < 0):
                    # Need UP
                    vz = 1480
                elif (d_alt > 0):
                    # Need Down
                    vz = 1478
                if vehicle.heading == need_heading:
                    if start_heading_home:
                        heading_home = True

                    yaw = 1500
                    pitch = 1000
                    right_direction = True
                    start_time = time.time()


                elif vehicle.heading - need_heading > 355:
                    yaw = 1521
                elif vehicle.heading >= need_heading + 20:
                    yaw = 1450
                elif vehicle.heading <= need_heading - 20:
                    yaw = 1550
                elif vehicle.heading >= need_heading + 1:
                    yaw = 1479
                elif vehicle.heading <= need_heading - 1:
                    yaw = 1521

            elif right_height and right_direction:

                if not heading_home:
                    fly_forward_time = time.time() - start_time
                    start_forward = True
                    if distance > needed_distance - 34:
                        need_heading = needed_heading + 180 if 0 <= need_heading < 180 else need_heading - 180
                        right_direction = False
                        start_heading_home = True

                    if d_alt < 0:
                        vz = 1475
                    elif d_alt > 0:
                        vz = 1472
                else:
                    if time.time() - start_time >= fly_forward_time:
                        pitch = 1500
                        if d_alt < 0:
                            vz = 1475
                        elif d_alt > 0:
                            vz = 1472

            vehicle.channels.overrides = {'2': pitch, '3': vz, '4': yaw,}
            time.sleep(timeframe)
            if start_forward:
                distance+= timeframe * vehicle.groundspeed

up_turn_go(15, 180, 100)