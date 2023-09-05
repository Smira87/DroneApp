import time
from threading import Timer
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

class Copter:
    def __init__(self):
        self.state = {'mav_alt': 0, 'vz': 0, 'pitch': 1500, 'yaw': 1500, 'need_alt': 0, 'need_heading': 0}

    def connect(self):
        print('+++++++++++++++++')
        print('CONNECTING !!!')
        print('+++++++++++++++++')
        self.vehicle = connect('tcp:127.0.0.1:5763')
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.state['need_heading'] = self.vehicle.heading
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
                self.state['mav_alt'] = self.vehicle.location.global_relative_frame.alt

            except:
                pass
            self.set_heading()
            self.vehicle.channels.overrides = {'2': self.state['pitch'], '3': self.state['vz'], '4': self.state['yaw']}
            Timer(0.8, self.update).start()
    def fly_up(self, need_alt):
        self.state['need_alt'] = need_alt
        while self.state['mav_alt'] < need_alt:
            self.set_vz_up()
            time.sleep(0.8)

    def set_vz_up(self):
        d_alt = self.state['mav_alt'] - self.state['need_alt']
        print(self.state['vz'])
        if (d_alt < -12):
            # Need UP
            self.state['vz'] = 1500
        if (-12 < d_alt < 0):
            # Need UP
            self.state['vz'] = 1480
        elif (d_alt > 0):
            # Need Down
            self.state['vz'] = 1478
    def set_heading(self):
        if self.vehicle.heading - self.state['need_heading'] > 355:
            self.state['yaw'] = 1521
        elif self.state['need_heading'] - self.vehicle.heading > 180:
            self.state['yaw'] = 1450
        elif self.vehicle.heading <= self.state['need_heading'] - 20 or\
                self.vehicle.heading - self.state['need_heading'] > 180:
            self.state['yaw'] = 1550
        elif self.vehicle.heading >= self.state['need_heading'] + 20:
            self.state['yaw'] = 1450
        elif self.vehicle.heading >= self.state['need_heading'] + 1:
            self.state['yaw'] = 1479
        elif self.vehicle.heading <= self.state['need_heading'] - 1:
            self.state['yaw'] = 1521
        elif self.vehicle.heading == self.state['need_heading']:
            self.state['yaw'] = 1500

    def turn(self, heading):
        self.state['need_heading'] = heading
        while self.vehicle.heading != heading:
            self.set_vz_up()

    def fly_forward(self, need_distance):
        distance = 0
        while distance < need_distance - 35:
            self.state['pitch'] = 1000
            d_alt = self.state['mav_alt'] - self.state['need_alt']
            if d_alt < 0:
                vz = 1475
            elif d_alt > 0:
                vz = 1472

