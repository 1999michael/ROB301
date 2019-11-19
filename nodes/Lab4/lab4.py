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
		self.st = 0
	

	def pid(self):
	    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	    st_sub = rospy.Subscriber('state',String,self.callback_state,queue_size=1)
	    twist=Twist()
	    twist.linear.x=0.1
	    twist.angular.z=0.0
	    #kp = 0.00936
	    #ki = 0.01783
	    #kd = 0.0012285
	    #kp = 0.00936
	    #ki = 0.0182
	    #kd = 0.0011
	    kp = 0.0078


	    rate = 20.0
	    r=rospy.Rate(rate)
	    color_sub = rospy.Subscriber('color_mono',String,self.callback,queue_size=10)
	    

	    destinations = [0.61, 1.22, 2.44, 3.05]
	    dcounter = 0
	    time_delay_count = 0
	    enable =1
	    while(True):
		print(self.st)
		if(enable==1)and(self.st>=destinations[dcounter]):
			twist.linear.x=0.0
    			twist.angular.z=0.0
			cmd_pub.publish(twist)
	    		if (time_delay_count <= rate*2):
				time_delay_count = time_delay_count + 1
			
			else:
				dcounter = dcounter + 1
				time_delay_count = 0
				if(dcounter==4):
					enable = 0
		else:
			twist.linear.x=0.1
	    		twist.angular.z=0.0
		    	cmd_pub.publish(twist)
		r.sleep() 
	    pass

	def callback(self, color_data):	
		self.max_column = int(color_data.data)
		return color_data.data
	
	def callback_state(self, state):
		self.st = float(state.data)
		return state.data

def main():
    try:
        rospy.init_node('motor')
	robot = lab3()
	robot.pid()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
