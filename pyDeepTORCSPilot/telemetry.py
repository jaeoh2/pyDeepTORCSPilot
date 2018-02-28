import os
import sys
import socket
import struct
from collections import OrderedDict

        

class Telemetry(object):
    # refer from : https://github.com/BOSSoNe0013/RacingGameTelemetry/blob/master/tools/telemetry.py
    # refer from : https://github.com/marsauto/europilot/blob/master/europilot/controllerstate.py
    def __init__(self, hostname='127.0.0.1', port):
        self.state = self.__init_dict()
        self.hostname = hostname
        self.port = port

    def __init_dict(self):
        """Initialize the values for each of the controller output"""
        d = OrderedDict()

        d['TIME'] = '0'
        d['LAP_TIME'] = '0'
        d['LAP_DISTANCE'] = '0'
        d['OVERALL_DISTANCE']= '0'
        d['X_POSITION'] = '0'
        d['Y_POSITION'] = '0'
        d['Z_POSITION'] = '0'
        d['SPEED_MPS'] = '0'
        d['X_VELOCITY'] = '0'
        d['Y_VELOCITY'] = '0'
        d['Z_VELOCITY'] = '0'
        d['X_ROLL'] = '0'
        d['Y_ROLL'] = '0'
        d['Z_ROLL'] = '0'
        d['X_PITCH'] = '0'
        d['Y_PITCH'] = '0'
        d['Z_PITCH'] = '0'
        d['SUSPENSION_POSITION_REAR_LEFT'] = '0'
        d['SUSPENSION_POSITION_REAR_RIGHT'] = '0'
        d['SUSPENSION_POSITION_FRONT_LEFT'] = '0'
        d['SUSPENSION_POSITION_FRONT_RIGHT'] = '0'
        d['SUSPENSION_VELOCITY_REAR_LEFT'] = '0'
        d['SUSPENSION_VELOCITY_REAR_RIGHT'] = '0'
        d['SUSPENSION_VELOCITY_FRONT_LEFT'] = '0'
        d['SUSPENSION_VELOCITY_FRONT_RIGHT'] = '0'
        d['WHEEL_VELOCITY_REAR_LEFT_MPS'] = '0'
        d['WHEEL_VELOCITY_REAR_RIGHT_MPS'] = '0'
        d['WHEEL_VELOCITY_FRONT_LEFT_MPS'] = '0'
        d['WHEEL_VELOCITY_FRONT_RIGHT_MPS'] = '0'
        d['THROTTLE'] = '0'
        d['STEER'] = '0'
        d['BRAKE'] = '0'
        d['CLUTCH'] = '0'
        d['GEAR'] = '0'
        d['G_FORCE_LATERAL'] = '0'
        d['G_FORCE_LONGITUDINAL'] = '0'
        d['LAP'] = '0'
        d['ENGINE_RPM'] = '0'
        d['TOTAL_LAPS'] = '0'
        d['TRACK_LENGTH'] = '0'
        d['RACE_TIME'] = '0'
        d['MAX_RPM'] = '0'
        d['IDLE_RPM'] = '0'
        d['MAX_GEARS'] = '0'
        
        d['DIST_5_RANGE_FINDER'] = '0'

        d['MPS_KMH_RATE'] = 3.59999999712

        d['GEAR_NEUTRAL'] = '0'
        d['GEAR_REVERSE'] = '0'

        # g27 state list
        d['wheel-axis'] = '0'
        d['clutch'] = '0'
        d['brake'] = '0'
        d['gas'] = '0'
        d['paddle-left'] = '0'
        d['paddle-right'] = '0'
        d['wheel-button-left-1'] = '0'
        d['wheel-button-left-2'] = '0'
        d['wheel-button-left-3'] = '0'
        d['wheel-button-right-1'] = '0'
        d['wheel-button-right-2'] = '0'
        d['wheel-button-right-3'] = '0'
        d['shifter-button-left'] = '0'
        d['shifter-button-right'] = '0'
        d['shifter-button-up'] = '0'
        d['shifter-button-down'] = '0'
        d['dpad-left/right'] = '0'
        d['dpad-up/down'] = '0'
        d['shifter-button-1'] = '0'
        d['shifter-button-2'] = '0'
        d['shifter-button-3'] = '0'
        d['shifter-button-4'] = '0'
        d['gear-1'] = '0'
        d['gear-2'] = '0'
        d['gear-3'] = '0'
        d['gear-4'] = '0'
        d['gear-5'] = '0'
        d['gear-6'] = '0'
        d['gear-R'] = '0'

        return d
        
    def update_state(self, msg):
        """Update ControllerState with the latest controller data"""
        k, v = msg.split()
        if k in self.state:
            self.state[k] = v
    
    def get_state(self):
        """Returns the latest state"""
        return self.state

    def connect(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((self.hostname, self.port))
        except Exception as e:
            sock.close()
    
    def parse(self, data):
        stats = struct.unpack('66f', data[0:264]) # TODO implement this method
