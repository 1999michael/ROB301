#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def publisher_node():
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist=Twist()
    twist.linear.x=0.1
    twist.angular.z=0.0
    last_time = time.time()
    now = 0
    r = rospy.Rate(10)
    while (now < 3.0):
	now = time.time() - last_time
	print(now)
        cmd_pub.publish(twist)
        r.sleep()
    print("Finish Linear")
    now = 0
    last_time = time.time()
    twist.linear.x = 0.0
    twist.angular.z = 0.1

    while(now < 3.0):
        now = time.time() - last_time
	print(now)
	cmd_pub.publish(twist)
        r.sleep()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    last_time = time.time()
    now = 0.0
    print("Finish Angular")
    while (now < 1.0):
        cmd_pub.publish(twist)
        now = time.time() - last_time
	print(now)
        r.sleep()
    pass


def main():
    
    try:
        rospy.init_node('motor')
	publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
