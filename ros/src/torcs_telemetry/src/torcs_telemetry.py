#!/usr/bin/env python
"""
# refer from :
# snakeoil.py
# Chris X Edwards <snakeoil@xed.ch>
# Snake Oil is a Python library for interfacing with a TORCS
# race car simulator which has been patched with the server
# extentions used in the Simulated Car Racing competitions.
# http://scr.geccocompetitions.com/
"""

import rospy
from std_msgs.msg import String, Header
from torcs_msgs.msg import TORCSSensors, TORCSCtrl
from sensor_msgs.msg import PointCloud2

import numpy as np
import socket
import struct
import os
import sys
import time

data_size = 2**17

def clip(v,lo,hi):
    if v<lo: return lo
    elif v>hi: return hi
    else: return v


def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
    if not s: return s
    # if type(s) is str:
    if type(s) is unicode:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]


class Client(object):
    def __init__(self, H=None, p=None, i=None, e=None, t=None, s=None, d=None, vision=False):
        # If you don't like the option defaults,  change them here.
        self.vision = vision

        self.host = 'localhost'
        self.port = 3001
        self.sid = 'SCR'
        self.maxEpisodes = 0 # "Maximum number of learning episodes to perform"
        self.trackname = 'unknown'
        self.stage = 3  # 0=Warm-up, 1=Qualifying 2=Race, 3=unknown <Default=3>
        self.debug = False
        self.maxSteps = 100000  # 50steps/second
        if H: self.host = H
        if p: self.port = p
        if i: self.sid = i
        if e: self.maxEpisodes = e
        if t: self.trackname = t
        if s: self.stage = s
        if d: self.debug = d

        self.pub = rospy.Publisher('torcs_sensors',
                              TORCSSensors,
                              queue_size=10
                              )
        self.sub = rospy.Subscriber('torcs_ctrl', TORCSCtrl, self.drive_control)
        self.rate = rospy.Rate(100)

        self.S = ServerState()
        self.R = DriverAction()

        self.setup_connection()

    def setup_connection(self):
        # == Set Up UDP Socket ==
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket...{}'.format(emsg))
            sys.exit(-1)
        # == Initialize Connection To Server ==
        self.so.settimeout(1)

        n_fail = 5
        while True:
            # This string establishes track sensor angles! You can customize them.
            # a= "-90 -75 -60 -45 -30 -20 -15 -10 -5 0 5 10 15 20 30 45 60 75 90"
            # xed- Going to try something a bit more aggressive...
            a = "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"

            # initmsg = '%s(init %s)' % (self.sid, a)
            initmsg = "{}(init {})".format(self.sid, a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata = str()
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print("Waiting for server on %d............" % self.port)

            identify = '***identified***'
            if identify in sockdata:
                print("Client connected on %d.............." % self.port)
                break


    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata = str()

        while True:
            try:
                # Receive server data
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                # print('.', end=' ')
                print("Waiting for data on %d.............." % self.port)
            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print((("Server has stopped the race on %d. " +
                        "You were in %d place.") %
                       (self.port, self.S.d['racePos'])))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                # What do I do here?
                print("Server has restarted the race on %d." % self.port)
                # I haven't actually caught the server doing this.
                self.shutdown()
                return
            elif not sockdata:  # Empty?
                continue  # Try again.
            else:
                self.S.parse_server_str(sockdata)

                tos = TORCSSensors()
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'torcs'
                tos.header = header
                tos.angle = self.S.d['angle']
                tos.currentLapTime = self.S.d['curLapTime']
                tos.damage = self.S.d['damage']
                tos.distFromStart = self.S.d['distFromStart']
                tos.distRaced = self.S.d['distRaced']
                tos.focus = self.S.d['focus']
                tos.fuel = self.S.d['fuel']
                tos.gear = self.S.d['gear']
                tos.lastLapTime = self.S.d['lastLapTime']
                tos.opponents = self.S.d['opponents']
                tos.racePos = self.S.d['racePos']
                tos.rpm = self.S.d['rpm']
                tos.speedX = self.S.d['speedX']
                tos.speedY = self.S.d['speedY']
                tos.speedZ = self.S.d['speedZ']
                tos.trackSensor = self.S.d['track']
                tos.trackPos = self.S.d['trackPos']
                tos.wheelSpinVel =self.S.d['wheelSpinVel']
                tos.z = self.S.d['z']

                self.pub.publish(tos)
                self.rate.sleep()

                # rospy.loginfo(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H")  # Clear for steady output.
                    print(self.S)
                break  # Can now return from this function.


    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))

        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1], str(emsg[0])))


    def drive_control(self, torcs_ctrl):
        if not self.so: return

        self.R['steer'] = torcs_ctrl.steering
        self.R['accel'] = torcs_ctrl.accel
        self.R['gear'] = torcs_ctrl.gear
        self.R['brake'] = torcs_ctrl.brake


    def shutdown(self):
        if not self.so: return
        print(("Race terminated or %d steps elapsed. Shutting down %d."
               % (self.maxSteps, self.port)))
        self.so.close()
        self.so = None
        # sys.exit() # No need for this really.


class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr= str()
        self.d= dict()

    def parse_server_str(self, server_string):
        '''Parse the server string.'''
        self.servstr= server_string.strip()[:-1]
        sslisted= self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w= i.split(' ')
            self.d[w[0]]= destringify(w[1:])


class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
    def __init__(self):
       self.actionstr= str()
       # "d" is for data dictionary.
       self.d= { 'accel':0.2,
                   'brake':0,
                  'clutch':0,
                    'gear':1,
                   'steer':0,
                   'focus':[-90,-45,0,45,90],
                    'meta':0
                    }

    def clip_to_limits(self):
        """There pretty much is never a reason to send the server
        something like (steer 9483.323). This comes up all the time
        and it's probably just more sensible to always clip it than to
        worry about when to. The "clip" command is still a snakeoil
        utility function, but it should be used only for non standard
        things or non obvious limits (limit the steering to the left,
        for example). For normal limits, simply don't worry about it."""
        self.d['steer']= clip(self.d['steer'], -1, 1)
        self.d['brake']= clip(self.d['brake'], 0, 1)
        self.d['accel']= clip(self.d['accel'], 0, 1)
        self.d['clutch']= clip(self.d['clutch'], 0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear']= 0
        if self.d['meta'] not in [0,1]:
            self.d['meta']= 0
        if type(self.d['focus']) is not list or min(self.d['focus'])<-180 or max(self.d['focus'])>180:
            self.d['focus']= 0

    def __repr__(self):
        self.clip_to_limits()
        out= str()
        for k in self.d:
            out+= '('+k+' '
            v= self.d[k]
            if not type(v) is list:
                out+= '%.3f' % v
            else:
                out+= ' '.join([str(x) for x in v])
            out+= ')'
        return out
        return out+'\n'


def drive_example(c):
    '''This is only an example. It will get around the track but the
    correct thing to do is write your own `drive()` function.'''
    S,R= c.S.d,c.R.d
    target_speed=100

    # Steer To Corner
    R['steer']= S['angle']*15 / np.pi
    # Steer To Center
    R['steer']-= S['trackPos']*.10

    # Throttle Control
    if S['speedX'] < target_speed - (R['steer']*50):
        R['accel']+= .01
    else:
        R['accel']-= .01
    if S['speedX']<10:
       R['accel']+= 1/(S['speedX']+.1)

    # Traction Control System
    if ((S['wheelSpinVel'][2]+S['wheelSpinVel'][3]) -
       (S['wheelSpinVel'][0]+S['wheelSpinVel'][1]) > 5):
       R['accel']-= .2

    # Automatic Transmission
    R['gear']=1
    if S['speedX']>50:
        R['gear']=2
    if S['speedX']>80:
        R['gear']=3
    if S['speedX']>110:
        R['gear']=4
    if S['speedX']>140:
        R['gear']=5
    if S['speedX']>170:
        R['gear']=6
    return


if __name__ == '__main__':
    try:
        rospy.init_node('torcs_telemetry', anonymous=True)
        client = Client(p=3001)

        while not rospy.is_shutdown():
            # ToDo: Implement server status publisher
            # rospy.spin()
            client.get_servers_input()
            drive_example(client)
            client.respond_to_server()
            # rate.sleep()

    except rospy.ROSInterruptException:
        pass



