from pygame import joystick
import pygame
import time
import serial


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

            self.turn(self.joy.get_axis(self.x_axis))
            self.sheet(self.joy.get_axis(self.throttle_axis))

            time.sleep(0.05)


control = KiteControl()
control.run()

