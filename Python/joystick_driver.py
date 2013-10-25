__author__ = 'Adrien'
from pygame import joystick
import pygame
import time

joystick.init()
pygame.display.init()

print 'joystick count: ', joystick.get_count()

joy = joystick.Joystick(0)
joy.init()

print 'num axes: ', joy.get_numaxes()
"""
Axis 0: (Left,Right) = (-32768, 32767), center = -1
Axis 1: (Forward,Backward) = (-32768, 32767), center = -1
Axis 2: Left throttle (Forward, Backward) = (-32768, 32767), center = -1
Axis 3: Twist (CCW, CW) = (-32768, 32767), center = -1
Axis 4: Right Throttle (Forward, Backward) = (-32768, 32767), center = -1
"""
print 'num buttons: ', joy.get_numbuttons()

while True:
    pygame.event.get()
    pygame.event.pump()
    print joy.get_axis(0)
    print "Left/Right Axis:", joy.get_axis(0)
    print "Forw/Backw Axis:", joy.get_axis(1)

    time.sleep(0.5)



