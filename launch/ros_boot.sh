#!/bin/bash
# file: that starts ros on boot
# needs to be started from contrab, check /catkin_ws/src/robosoc_eurobot_2017/src/LCD/display_wifi.py for details

 
source /opt/ros/kinetic/setup.bash
 
roscore
