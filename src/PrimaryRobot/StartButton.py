#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO  
GPIO.setmode(GPIO.BCM)

# GPIO 23 set up as input. It is pulled up to stop false signals  
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)  

def talker():
    pub = rospy.Publisher('startButton', String, queue_size=10)
    rospy.init_node('start_button_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:  
            GPIO.wait_for_edge(17, GPIO.FALLING)  
            print "\nFalling edge detected. Now your program can continue with"  
            print "whatever was waiting for a button press."
            hello_str = "Button pressed %s" % rospy.get_time()  
        except KeyboardInterrupt:  
            GPIO.cleanup()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
