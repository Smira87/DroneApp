import time
from threading import Timer
from dronekit import connect, VehicleMode
from pymavlink import mavutil

class Copter:
    timeframe = 0.1
    def __init__(self):
        self.state = {'mav_alt': 0, 'rc_throttle': 0, 'rc_pitch': 1500, 'rc_yaw': 1500, 'need_alt': 0, 'need_heading': 0}
        self.fast_up = 1900
        self.middle_up = 1700
        self.slow_up = 1620
        self.hold_up = 1500
    def connect(self):
        print('+++++++++++++++++')
        print('CONNECTING !!!')
        print('+++++++++++++++++')
        self.vehicle = connect('tcp:127.0.0.1:5763')
        self.vehicle.mode = VehicleMode("ALT_HOLD")
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
            print(self.state['rc_throttle'])
            self.vehicle.channels.overrides = {'2': self.state['rc_pitch'], '3': self.state['rc_throttle'], '4': self.state['rc_yaw']}
            Timer(self.timeframe, self.update).start()

    def fly_up(self, need_alt):
        self.state['need_alt'] = need_alt
        while self.state['mav_alt'] < need_alt:
            d_alt = self.state['mav_alt'] - self.state['need_alt']

            if (d_alt < -5):
                # Need UP
                self.state['rc_throttle'] = self.fast_up
            elif (-5 < d_alt < -2):
                # Need UP
                self.state['rc_throttle'] = self.middle_up
            elif (-2 < d_alt < 0):
                # Need UP
                self.state['rc_throttle'] = self.slow_up

            time.sleep(0.8)
        self.state['rc_throttle'] = self.hold_up


    def set_heading(self):
        if self.vehicle.heading and self.vehicle.heading - self.state['need_heading'] > 355:
            self.state['rc_yaw'] = 1521
        elif self.vehicle.heading and self.state['need_heading'] - self.vehicle.heading > 180:
            self.state['rc_yaw'] = 1450
        elif self.vehicle.heading and self.vehicle.heading <= self.state['need_heading'] - 20 or\
                self.vehicle.heading - self.state['need_heading'] > 180:
            self.state['rc_yaw'] = 1550
        elif self.vehicle.heading and self.vehicle.heading >= self.state['need_heading'] + 20:
            self.state['rc_yaw'] = 1450
        elif self.vehicle.heading and self.vehicle.heading >= self.state['need_heading'] + 1:
            self.state['rc_yaw'] = 1479
        elif self.vehicle.heading and self.vehicle.heading <= self.state['need_heading'] - 1:
            self.state['rc_yaw'] = 1521
        elif self.vehicle.heading and self.vehicle.heading == self.state['need_heading']:
            self.state['rc_yaw'] = 1500
