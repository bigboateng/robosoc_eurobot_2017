#!/usr/bin/env python

"""
This code will alert main program if robot is near obstacle/wall
"""
import rospy
from std_msgs.msg import String
from ..Sensors.SRF08 import SRF08


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
    # Initialize the 3 sensors
    sensor_front_left = SRF08()
    sensor_front_right = SRF08()
    sensor_back = SRF08() # todo(boateng): this is IR!

    sensor_threshold = 0.1
    ir_threshold = 0.1

    collision_sensors = []
    while not rospy.is_shutdown():
        if sensor_front_left.get_distance() > sensor_threshold:
            collision_sensors.append("front_left")

        if sensor_front_right.get_distance() > sensor_threshold:
            collision_sensors.append("front_right")

        if sensor_back.get_distance() > sensor_threshold:
            collision_sensors.append("back")

        if len(collision_sensors) > 0:
            obstacle_alert.publish(collision_sensors)

        rate.sleep()


if __name__ == "__main__":
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
