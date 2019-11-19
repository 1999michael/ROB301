#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def publisher_node():
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist=Twist()
    twist.linear.x=0.2
    twist.angular.z=0.0
    last_time = time.time()
    curr_time = 0
    r = rospy.Rate(10)	
    last_time = time.time()

    # Execute straight line motion
    while (curr_time < 5.0):
	curr_time = time.time() - last_time
	print(curr_time)
        cmd_pub.publish(twist)
        r.sleep()

    curr_time = 0
    last_time = time.time()

    # Execute on the spot 90 degree turn
    twist.linear.x = 0.0
    twist.angular.z = 0.2

    while(now < 7.85398):
        now = time.time() - last_time
	print(now)
	cmd_pub.publish(twist)
        r.sleep()

    # Stop the robot
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_pub.publish(twist)
    pass


def get_yaw_from_quarternion(q):
    siny_cosp = 2* (q.w*q.z + q.x*q.y)
    cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
    yaw = math.atan(siny_cosp/cosy_cosp)
    return yaw
    rospy.loginfo(cur_pose)
    pass

def callback(odom_data):
    point=odom_data.pose.pose.position
    quart=odom_data.pose.pose.orientation
    theta=get_yaw_from_quarternion(quart)
    cur_pose = (point.x, point.y, theta)
    return  cur_pose


def pose_to_pose():
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    twist = Twist()
    twist.linear.x=0.1
    twist.angular.z=0.0
    r = rospy.Rate(50)
    odom_sub=rospy.Subscriber('odom',Odometry,callback,queue_size=10)
    print(odom_sub.data)

def main():

    # Make robot move along a square path

    try:
        rospy.init_node('motor')
        publisher_node()
        publisher_node()
        publisher_node()
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
