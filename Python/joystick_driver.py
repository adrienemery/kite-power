__author__ = 'Adrien'

from pygame import joystick
import time

joystick.init()

print 'joystick count: ', joystick.get_count()

joy = joystick.Joystick(0)
joy.init()

print 'num axes: ', joy.get_numaxes()
print 'num buttons: ', joy.get_numbuttons()

while True:
    print joy.get_axis(0), joy.get_axis(1), joy.get_axis(2), joy.get_axis(3), joy.get_axis(4)
    time.sleep(0.1)