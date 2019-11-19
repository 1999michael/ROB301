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
    twist.linear.x=0.0
    twist.angular.z=0.25
    last_time = time.time()
    now = 0
    r = rospy.Rate(10)	
    last_time = time.time()
    while (now < 2.0):
	now = time.time() - last_time
	print(now)
        cmd_pub.publish(twist)
        r.sleep()
    print("Finish Turn")
    now = 0
    last_time = time.time()
    twist.linear.x = 0.2
    twist.angular.z = 0.0

    while(now < 4.12*2.5):
        now = time.time() - last_time
	print(now)
	cmd_pub.publish(twist)
        r.sleep()
    twist.linear.x = 0.0
    twist.angular.z = 0.25
    last_time = time.time()
    now = 0.0
    print("Finish Angular")
    while (now < 20.1/2.5):
        cmd_pub.publish(twist)
        now = time.time() - last_time
	print(now)
        r.sleep()
    twist.angular.z =0.0
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
    print("lalalal")
    odom_sub=rospy.Subscriber('odom',Odometry,callback,queue_size=10)
    
    print(odom_sub.data)

def main():
    
    try:
        rospy.init_node('motor')
	publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
