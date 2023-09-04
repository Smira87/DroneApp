import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

class Copter:
    def __init__(self):
        self.armed = 0
    def connect(self):
        print('+++++++++++++++++')
        print('CONNECTING !!!')
        print('+++++++++++++++++')
        self.vehicle = connect('tcp:127.0.0.1:5763')
        self.vehicle.mode = VehicleMode("STABILIZE")
        print('+++++++++++++++++')
        print('CONNECTED !!!')
        print('+++++++++++++++++')
    def arm(self):
        print('+++++++++++++++++')
        print('ARM !!!')
        print('+++++++++++++++++')
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,  # confirmation
            1,  # param 1
            0,  # param 2
            0,  # param 3
            0,  # param 4
            0, 0, 0)  # param 5 ~ 7 not used
        self.vehicle.send_mavlink(msg)
        time.sleep(3)
        print('+++++++++++++++++')
        print('ARMED !!!')
        print('+++++++++++++++++')

    def fly_up(self, need_alt):
        if (self.vehicle.armed):
            while True:
                try:
                    mav_alt = self.vehicle.location.global_relative_frame.alt
                except:
                    pass
                if (mav_alt is None): mav_alt = 0
                print('+++++++++++++++++++++++++')
                print(mav_alt)
                print('======= GO UP !!! =======')
                print('+++++++++++++++++++++++++')

                d_alt = mav_alt - need_alt

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

                self.vehicle.channels.overrides = {'3': vz}
                time.sleep(0.8)