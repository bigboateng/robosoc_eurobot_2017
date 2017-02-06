#!/bin/sh
#/etc/init.d/myService
export PATH=$PATH:/usr/local/bin
export NODE_PATH=$NODE_PATH:/usr/local/lib/node_modules

case "$1" in
start)
exec forever --sourceDir=/home/pi/ros_catkin_ws/src/robosoc_eurobot_2017/src/server -p /home/pi/ros_catkin_ws/src/robosoc_eurobot_2017/src/server server.js  #scriptarguments
;;
stop)
exec forever stop --sourceDir=/home/pi/ros_catkin_ws/src/robosoc_eurobot_2017/src/server server.js
;;
*)
echo "Usage: /etc/init.d/ipService {start|stop}"
exit 1
;;
esac
exit 0