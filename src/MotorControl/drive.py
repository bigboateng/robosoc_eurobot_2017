from MD25 import MD25
import tty, time, termios, fcntl, sys, os
from Robot import Robot
import math

secondary_robot = Robot(0.181, 0.1)
controller = MD25(0x58,1,True,secondary_robot)
controller.resetEncoders()
class _GetchUnix:
  def __init__(self):
    import tty, sys

  def __call__(self):
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

getch = _GetchUnix()
k = ''
while k != 'x':
  k = getch()
  if k == 's':
    controller.forward(100)
  elif k == 'w':
    controller.forward(100)
  elif k == 'd':
    controller.turn(190,1)
  elif k == 'a':
    controller.turn(1,190)
  else:
    controller.stop()
  time.sleep(0.1)
  controller.stop()
  #controller.updatePosition()
  #x = controller.getXPosition()*100
  #y = controller.getYPosition()*100
  #theta = controller.getTheta()
  #print("X = {}, Y = {}, Theta = {}".format(x, y, math.degrees(theta)))	
