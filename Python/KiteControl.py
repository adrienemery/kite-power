__author__ = 'Adrien'
from pygame import joystick
import pygame
import time
import serial


def write_to_serial(msg):
    if ser.isOpen():
        if not '/' in msg:
            msg += '/'
        ser.write(msg)  # Terminating char '/' must always be appended to the end of the msg
        return True
    else:
        print 'Serial not available: ' + msg
        return False


def turn(val):
    val *= 30
    val = str(val)  # convert to string
    return write_to_serial('t' + val)


def sheet(val):
    val = (val + 1)/2 * 96  # maps the values (-1,1) to (0,96) This might need to be reversed?
    val = str(val)
    return write_to_serial('p' + val)


port = 'COM4'
baud = 9600

try:
    ser = serial.Serial(port, baud)
    print "Successfully opened Serial"
except serial.SerialException:
    print "Could not open Serial!"

# Initialize Joystick object
joystick.init()
pygame.display.init()
joy = joystick.Joystick(0)
joy.init()

x_axis = 0
throttle_axis = 4

while True:
    pygame.event.get()
    pygame.event.pump()

    turn(joy.get_axis(x_axis))
    sheet(joy.get_axis(throttle_axis))

    time.sleep(0.2)





