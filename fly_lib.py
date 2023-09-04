import time
from threading import Timer
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

class Copter:
    def __init__(self):
        self.armed = 0
        self.mav_alt = 0
        self.vz = 0
        self.pitch = 1500
        self.yaw = 1500
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

    def update(self):
        if (self.vehicle.armed):

            try:
                self.mav_alt = self.vehicle.location.global_relative_frame.alt

            except:
                pass

            self.vehicle.channels.overrides = {'2': self.pitch, '3': self.vz, '4': self.yaw,}
            print(self.vz)
            Timer(0.8, self.update).start()
    def fly_up(self, need_alt):

        while True:
            d_alt = self.mav_alt - need_alt
            print(d_alt)
            if (d_alt < -12):
                # Need UP
                self.vz = 1500
            elif (-5 < d_alt < 0):
                # Need UP
                self.vz = 1480
            elif (d_alt > 0):
                # Need Down
                self.vz = 1478
            time.sleep(0.8)
