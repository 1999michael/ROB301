#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0

        self.u = 0 # initialize the cmd_vel input
        self.phi = 22.8 #initialize the measurement input
        self.phi_exp = 22.8
        self.state_pub = rospy.Publisher('state', String, queue_size = 1)
	self.covariances = []

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
	#print(float(data.data))
	if (not math.isnan(float(data.data))):	
	    self.phi = float(data.data)*math.pi/180

    ## call within run_kf to update the state with the measurement 
    def predict(self, u = 0):
        self.x = self.x + (1.0/30.0)*u        
	return 

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self):
	if (self.d - self.x > 0):
        	self.phi_exp = np.arctan(self.h / (self.d-self.x))
	else:

		self.phi_exp = np.arctan(self.h / (self.d - self.x)) + math.pi
        return

    def run_kf(self):
        current_input = self.u
	#print current_input
        current_measurement = self.phi

	
	self.predict(current_input)
	self.measurement_update()

	D = self.h / ((self.h**2) + (self.d - self.x)**2)
        self.P = self.P + self.Q
	S = D*self.P*D + self.R
	W = self.P*D/S
	self.P = self.P - W*S*W
	print (current_measurement - self.phi_exp)
	self.x = self.x + W*(current_measurement - self.phi_exp)
        
	self.covariances.append(self.P)
        self.state_pub.publish(str(float(self.x)))
	if (self.x >= 3.10):
		return True


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    try:
        h = 0.6 #y distance to tower
        d = 1.5 #x distance to tower (from origin)  
        
        x_0 = 0 #initial state position
        
        Q = 0.01 #process noise covariance
        R = 0.01 #measurement noise covariance
        P_0 = 0.1 #initial state covariance 
        kf = KalmanFilter(h, d, x_0, Q, R, P_0)
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        rospy.sleep(1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            check = kf.run_kf()
	    if (check):
		plt.plot(kf.covariances)
		plt.savefig('covariances.png')  
            rate.sleep()
            
    #except:
        #print(e)

    finally:
        rospy.loginfo("goodbye")

