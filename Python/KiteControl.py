from pygame import joystick
import pygame
import time
import serial
from PID import PID
from KiteTracker import KiteTracker


class KiteControl(object):

    def __init__(self):
        self.port = 'COM4'
        self.baud = 9600

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
        self.joy = joystick.Joystick(0)
        self.joy.init()

        self.x_axis = 0  # this is usually 0
        self.throttle_axis = 1  # change this to the correct axis for the joystick

        self.last_turn_val = 0
        self.last_sheet_val = 47

        self.auto_enabled = False
        self.kite_tracker = KiteTracker()

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
                self.turn(self.joy.get_axis(self.x_axis))
                self.sheet(self.joy.get_axis(self.throttle_axis))
                self.kite_tracker.update()

            else:
                self.kite_tracker.update()  # must be called once per loop
                error = self.update_error()
                output = self.pid.update(error)
                self.turn(output)

            time.sleep(0.05)

    def update_error(self):
        """
        Input: tuple of x,y coordinates of the kite.
        Computes angle error between current heading and desired heading.
        Returns: error angle in degrees.
        """

        pos = self.kite_tracker.get_pos()

        # TODO get heading from kite position data and compute error

        return 0

control = KiteControl()
control.run()

