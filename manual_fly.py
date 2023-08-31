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
mav_yaw = 0
need_yaw = 0.1

vz = 1000
yaw = 1500
pitch = 1500

# ==================
vz_up_fast = 1540 

vz_hold = 1495

vz_hold_up = 1499
vz_hold_down = 1491

vz_ballance_up = 1504
vz_ballance_down = 1486

check_alt_ok = 0
alt_ok = 0
# =========================

yaw_hold = 1500

yaw_hold_up = 1510
yaw_hold_down = 1490

yaw_ballance_up = 1550
yaw_ballance_down = 1450

check_yaw_ok = 0
yaw_ok = 0

# =========================
pitch_time = 100
pitch_go = 1400

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
            mav_yaw = vehicle.attitude.yaw
        except:
            pass
        if (mav_alt is None): mav_alt = 0
        if (mav_yaw is None): mav_yaw = 0


        d_alt = mav_alt - need_alt
        d_yaw = mav_yaw - need_yaw
        print('+++++++++++++++++++++++++')
        print('Alt: '+str(mav_alt))
        print('Yaw: '+str(mav_yaw))
        print('+++++++++++++++++++++++++')
        
        print(str(mav_alt)+' + '+str(need_alt)+' = '+str(d_alt))



        # ==========================================
        if (d_alt <= -12):
            # Fast UP!
            vz = vz_up_fast
        elif (d_alt > -12 and vz == vz_up_fast):
            # Slow UP!
            vz = vz_ballance_up
            action = 'fly_stab'
            # alt_ok = 1 # for fast test

        # =============== Hold Ballance ====================
        elif (d_alt > -0.2 and d_alt < 0.2):
            vz = vz_hold
            check_alt_ok += 1
        # ================== Fix near Ballance Up  ==================
        elif (d_alt <= -0.2 and d_alt >= -1):
            vz = vz_hold_up

        # ================== Fix near Ballance Down  ==================
        elif (d_alt >= 0.2 and d_alt <= 1):
            vz = vz_hold_down
            check_alt_ok += 1

        # ================== Fix long Ballance Down  ==================
        elif (d_alt > 1):
            vz = vz_ballance_down
            check_alt_ok = 0

        # ================== Fix long Ballance Up  ==================
        elif (d_alt < -1):
            vz = vz_ballance_up
            check_alt_ok = 0


        if (check_alt_ok > 10):
            alt_ok = 1

        if (alt_ok == 1):
            print('>>>>>>> ALT - OK!') # >>>>>>>>> YAW CHANGE !!!

            # =============== Hold Ballance ====================
            if (d_yaw > -0.02 and d_yaw < 0.02):
                d_yaw = yaw_hold
                check_yaw_ok += 1
            # ================== Fix near Ballance Up  ==================
            elif (d_yaw <= -0.02 and d_yaw >= -0.1):
                yaw = yaw_hold_up

            # ================== Fix near Ballance Down  ==================
            elif (d_yaw >= 0.02 and d_yaw <= 0.1):
                yaw = yaw_hold_down
                check_yaw_ok += 1

            # ================== Fix long Ballance Down  ==================
            elif (d_yaw > 0.1):
                yaw = yaw_ballance_down
                check_yaw_ok = 0

            # ================== Fix long Ballance Up  ==================
            elif (d_yaw < -0.1):
                yaw = yaw_ballance_up
                check_yaw_ok = 0


            if (check_yaw_ok > 10):
                yaw_ok = 1

            if (yaw_ok == 1):
                print('>>>>>>> YAW - OK!') # >>>>>>>>> GO !!!

                if (pitch_time > 0):
                    print('>>>>>>> GO FORWARD!')
                    pitch = pitch_go
                    pitch_time -= 1
                else:
                    print('>>>>>>> STOP!')
                    pitch = 1500

         
        print('====== P: '+str(pitch)+'====== Up: '+str(vz)+'====== Angle: '+str(yaw))
        vehicle.channels.overrides = {'2': pitch, '3': vz, '4': yaw}
        time.sleep(0.5)