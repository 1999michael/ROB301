#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# ku = 0.014
#actual ku = 0.0156
# Tu = 1.08
#actual Tu = 1.05
class lab3():
	def __init__(self):
	    self.max_column = 0
	
	def bang_bang(self):
	    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	    twist=Twist()
	    twist.linear.x=0.1
	    twist.angular.z=0.0
	    r=rospy.Rate(20)
	    color_sub = rospy.Subscriber('color_mono',String,self.callback,queue_size=10)
	    while(True):
		print(self.max_column)
		diff = self.max_column - 320
		if (diff > 0):
		    twist.linear.x = 0.1
		    twist.angular.z = -0.2
		    if (diff > 100):
		    	twist.linear.x = 0
		elif (diff < 0):
		    twist.linear.x = 0.1
		    twist.angular.z = 0.2
		    if (diff < -100):
		    	twist.linear.x = 0
	    	cmd_pub.publish(twist)
		r.sleep() 
	    pass


	def p(self):
	    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	    twist=Twist()
	    twist.linear.x=0.1
	    twist.angular.z=0.0
	    kp = 0.007
	    r=rospy.Rate(20)
	    color_sub = rospy.Subscriber('color_mono',String,self.callback,queue_size=10)
	    while(True):
		
		diff = self.max_column - 320
		correction = -kp*diff
		print(correction)
		twist.angular.z= correction 
	    	cmd_pub.publish(twist)
		r.sleep() 
	    pass


	def pi(self):
	    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	    twist=Twist()
	    twist.linear.x=0.1
	    twist.angular.z=0.0
	    kp = 0.007
	    ki = 0.008
	    rate = 20.0
	    r=rospy.Rate(rate)
	    integral = 0
	    icap = 0.025 / ki
	    color_sub = rospy.Subscriber('color_mono',String,self.callback,queue_size=10)
	    while(True):
		diff = 320.0 - self.max_column 
		integral = integral + (diff / rate)
		if (abs(ki*integral) >= icap):
		    if (integral > 0):
		    	integral = icap / ki
		    else:
			integral = -icap / ki
		correction = kp*diff + ki*integral
		twist.angular.z= correction 
		print("kp: ", kp*diff, " ki: ", ki*integral," Diff: ", self.max_column)
	    	cmd_pub.publish(twist)
		r.sleep() 
	    pass


	def pid(self):
	    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	    twist=Twist()
	    twist.linear.x=0.1
	    twist.angular.z=0.0
            # Ziegler-Nichols Gains (based off ultimate gain and ultimate period)
	    #kp = 0.00936
	    #ki = 0.01783
	    #kd = 0.0012285
	    kp = 0.00936
	    ki = 0.0182
	    kd = 0.0011
	    icap = 0.01/ ki

	    rate = 20.0
	    r=rospy.Rate(rate)
	    color_sub = rospy.Subscriber('color_mono',String,self.callback,queue_size=10)
	    prev = 320.0 - self.max_column 
	    integral = 0
	    while(True):
		print(self.max_column)
		diff = 320.0 - self.max_column 
		integral = integral + (diff / rate)
		if (abs(ki*integral) >= icap):
		    if (integral > 0):
		    	integral = icap / ki
		    else:
			integral = -icap / ki
		derivative = (diff - prev) * rate
		prev = diff
		correction = kp*diff + ki*integral + kd*derivative
		twist.angular.z= correction 
	    	cmd_pub.publish(twist)
		print("kp: ", kp*diff, "ki: ", ki*integral,"kd: ", kd*derivative)
		r.sleep() 
	    pass

	def callback(self, color_data):		
	    self.max_column = int(color_data.data)
	    return color_data.data


def main():
    try:
        rospy.init_node('motor')
	robot = lab3()
	robot.pid()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
