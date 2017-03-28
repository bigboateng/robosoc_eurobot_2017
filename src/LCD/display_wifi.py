#!/usr/bin/env python
import I2C_LCD_driver
import socket
import fcntl
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915, 
        struct.pack('256s', ifname[:15])
    )[20:24])

mylcd = I2C_LCD_driver.lcd()
mylcd.lcd_display_string("IP Address:", 1) 
mylcd.lcd_display_string(get_ip_address('wlan0'), 2)

#script runs from cron using @reboot
#running crontab -e allows to edit cron
#this is actual code from crontab:
# #LCD showing IP address
# @reboot /home/pi/catkin_ws/src/robosoc_eurobot_2017/src/LCD/display_wifi.py
# this script needs to be in this folder

