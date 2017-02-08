from MD25 import MD25
import tty, time, termios, fcntl, sys, os

controller = MD25(0x58,1,True)
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
  one = controller.readEncoder(type=1)
  two = controller.readEncoder(type=2)
  print("Encoder 1 = {}, Encoder 2 = {}".format(one, two))	
