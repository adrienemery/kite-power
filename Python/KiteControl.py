from pygame import joystick
import pygame
import time
import serial
import numpy as np
from PID import PID
from KiteTracker import KiteTracker


class KiteControl(object):

    def __init__(self):
        self.port = 'COM22'
        self.baud = 115200

        try:
            self.ser = serial.Serial(self.port, self.baud)
            print "Successfully opened Serial"
            print
        except serial.SerialException:
            self.ser = None
            print "Could not open Serial!"
            print

        # Initialize Joystick object
        joystick.init()
        pygame.display.init()

        if joystick.get_count():
            self.joy = joystick.Joystick(0)
            self.joy.init()

        else:
            self.joy = None

        self.x_axis = 0  # this is usually 0
        self.throttle_axis = 1  # change this to the correct axis for the joystick

        self.last_turn_val = 0
        self.last_sheet_val = 47

        self.auto_enabled = True
        self.kite_tracker = KiteTracker()
        self.pos_list = []

        self.pid = PID(3.0, 0.4, 1.2)
        self.pid.setSetPoint(0.0)  # Set-point corresponds to heading vector which has angle = 0.0 in its frame.

    def write_to_serial(self, msg):
        if self.ser:
            if not '/' in msg:
                msg += '/'
            self.ser.write(msg)  # Terminating char '/' must always be appended to the end of the msg
            return True
        else:
            if not '/' in msg:
                msg += '/'
            print 'Serial not available: ' + msg
            return False

    def turn(self, val):
        val *= 30
        val = int(val)
        if not val == self.last_turn_val:
            self.last_turn_val = val
            return self.write_to_serial('t' + str(val))

    def sheet(self, val):
        val = (val + 1)/2 * 96  # maps the values (-1,1) to (0,96) This might need to be reversed?
        val = int(val)

        if not val == self.last_sheet_val:
            self.last_sheet_val = int(val)
            return self.write_to_serial('p' + str(val))

    def run(self):

        while True:
            pygame.event.get()
            pygame.event.pump()

            if not self.auto_enabled:
                if self.joy:
                    self.turn(self.joy.get_axis(self.x_axis))
                    self.sheet(self.joy.get_axis(self.throttle_axis))

                self.kite_tracker.update()

            else:
                self.kite_tracker.update()  # must be called once per loop
                error = self.update_error()
                #output = self.pid.update(error)
                #self.turn(output)

            time.sleep(0.05)

    def update_error(self):
        """
        Input: tuple of x,y coordinates of the kite.
        Computes angle error between current heading and desired heading.
        Returns: error angle in degrees.
        """

        # get updated position from kite tracker and store in array
        self.pos_list.append(self.kite_tracker.get_pos())

        if len(self.pos_list) > 3:
            last_pos = self.pos_list[-3]
            pos = self.pos_list[-1]
            target = self.kite_tracker.get_current_target()

            if pos and last_pos and last_pos != pos:

                # only keep the last 5 positions
                if len(self.pos_list) > 5:
                    self.pos_list = self.pos_list[1:]

                # compute the heading based on the last average vector over the last 2 positions

                vecA = np.array([pos[0]-last_pos[0], pos[1] - last_pos[1]])
                vecA = vecA/np.linalg.norm(vecA)
                vecB = np.array([target[0] - last_pos[0], target[1] - last_pos[1]])
                vecB = vecB/np.linalg.norm(vecB)

                ctheta = vecA.dot(vecB)/(np.linalg.norm(vecA) * np.linalg.norm(vecB)) # = cos(theta)
                if ctheta > 1:
                    ctheta = 1.0

                theta = np.rad2deg(np.arccos(ctheta))  # original unsigned angle

                angle = 1
                rot = np.mat([[np.cos(np.deg2rad(angle)), -np.sin(np.deg2rad(angle))],
                              [np.sin(np.deg2rad(angle)), np.cos(np.deg2rad(angle))]])

                # rotate vecA by 1 degree ccw
                rot_vecA = rot * np.mat(vecA).T
                rot_vecA = rot_vecA.T

                # recalcluate angle
                ctheta = rot_vecA.dot(vecB)/(np.linalg.norm(rot_vecA) * np.linalg.norm(vecB))
                if ctheta > 1:
                    ctheta = 1.0

                theta_new = np.rad2deg(np.arccos(ctheta))  # original unsigned angle

                if theta_new < theta:
                    theta = -theta

                print theta
                return theta
            else:
                print 0
                return 0

        else:
            print 0
            return 0

if __name__ == '__main__':

    control = KiteControl()
    control.run()

