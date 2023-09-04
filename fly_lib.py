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

