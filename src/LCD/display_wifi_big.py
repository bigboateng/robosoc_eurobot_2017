#!/usr/bin/env python
# loading the class
import lcddriver
from time import *
import socket
import fcntl
import struct

# lcd start
lcd = lcddriver.lcd()

# this command clears the display (captain obvious)
lcd.lcd_clear()

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915, 
        struct.pack('256s', ifname[:15])
    )[20:24])

# now we can display some characters (text, line)
lcd.lcd_display_string("   You looser   ", 1)
lcd.lcd_display_string("     Fuck you", 2)
lcd.lcd_display_string(" with love xx, IP:", 3)
lcd.lcd_display_string(" test", 4)

#script runs from cron using @reboot
#running crontab -e allows to edit cron
#this is actual code from crontab:
# #LCD showing IP address
# @reboot /home/pi/catkin_ws/src/robosoc_eurobot_2017/src/LCD/display_wifi.py
# this script needs to be in this folder

