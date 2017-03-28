#!/usr/bin/env python
import I2C_LCD_driver
import socket
import fcntl
import struct
import rospy
from std_msgs.msg import String

mylcd = I2C_LCD_driver.lcd()

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915, 
        struct.pack('256s', ifname[:15])
    )[20:24])


def message_received(msg):
	arr = msg.data.split(",")
	title = arr[0]
	message = arr[1]
	mylcd.lcd_display_string(title, 1) 
	mylcd.lcd_display_string(message, 2) 

def init_node():
	rospy.init_node("lcd_display_node")
	sub = rospy.Subscriber("lcd_node", String, message_received)
	while not rospy.is_shutdown():
		rospy.spin
#http://www.circuitbasics.com/raspberry-pi-i2c-lcd-set-up-and-programming/
mylcd.lcd_display_string("IP Address:", 1) 

mylcd.lcd_display_string(get_ip_address('wlan0'), 2)

if __name__ == '__main__':
	try:
		init_node()
	except rospy.ROSInterruptException:
		pass
