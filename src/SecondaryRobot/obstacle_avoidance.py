#!/usr/bin/env python

"""
This code will alert main program if robot is near obstacle/wall
"""
import rospy
from std_msgs.msg import String
from os import sys, path
sys.path.append('../UltrasonicSensors')
from SRF08 import SRF08
sys.path.append('../IRSensors')
from IRsensor import IRsensor
import time

def initialize_node():
    """
    Setting up ros node and topic
    """
    rospy.init_node("obstacle_avoidance", anonymous=True)
    # The publisher to send obstacle alert
    obstacle_alert = rospy.Publisher("obstacle_alert", String, queue_size=10)
    # How fast to check for obstacles per second
    refresh_rate = 10 #Hz
    rate = rospy.Rate(refresh_rate)
    # SPI/IR Sensor
    SPI_PORT   = 0
    SPI_DEVICE = 0
    channel_mcp_IR_sensor_back = 0 # backward sensor 
    IR_sensor_back = IRsensor(SPI_PORT, SPI_DEVICE, channel_mcp_IR_sensor_back)
    # Initialize the Ultrasonic sensor
    address_i2c_US_sensor_front = 0xE0 >> 1
    US_sensor_front = SRF08(address_i2c_US_sensor_front, 2)


    ut_threshold = 0.1
    ir_threshold = 0.1

    alert_message = ""
    while not rospy.is_shutdown():
        IR_sensor_back.getAnalogValue()
        if IR_sensor_back.distance > ir_threshold:
            alert_message = "front"
            obstacle_alert.publish(alert_message)
        if sensor_back.US_sensor_front() > ut_threshold:
            alert_message = "back"
            obstacle_alert.publish(alert_message)
        rate.sleep()


if __name__ == "__main__":
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
