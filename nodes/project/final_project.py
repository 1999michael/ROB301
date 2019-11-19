#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

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


class BayesLoc:

    def __init__(self, color_map):
        self.colour_sub = rospy.Subscriber('mean_img_rgb', String, self.measurement_callback)
        self.line_idx_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.color_map = color_map
        self.measured_rgb = np.array([0,0,0]) # updated with the measurement_callback
        self.line_idx = 0 # updated with the line_callback with the index of the detected black line.
        
	# MAP AND MODEL PARAMETERS 
	self.topological_map = {0: "green", 1: "orange", 2: "green", 3: "yellow", 4: "blue", 5: "green", 6: "orange",7:"yellow", 8: "blue", 9: "blue", 10: "orange", 11: "yellow"}
        self.num_states = len(self.topological_map)
        self.control_model = {-1: [0.85, 0.10, 0.05], 0:[0.05, 0.90, 0.05], 1:[0.05, 0.10, 0.85]}
        self.colors = ["blue", "green", "yellow", "orange", "black"]
        self.measurement_model = {"blue": [0.6, 0.2, 0.05, 0.05, 0.1], "green": [0.2, 0.6, 0.05, 0.05, 0.1], "yellow":[0.05, 0.05, 0.65, 0.15, 0.1], "orange":[0.05, 0.05, 0.2, 0.6, 0.1]}
        self.probabilities_location = [[1.0/self.num_states for i in range(self.num_states)]]
	# black = 0, green = 1, orange = 2, blue = 3, yellow = 4
 
    # === BAYESIAN LOCALIZATION ===
    def update(self, control, measurement):
        #measurement = self.colors.index(measurement)
        if(control == None or measurement == None):
            return True

        # === STATE PREDICTION ===
        curr_probabilities = self.probabilities_location[-1]
        pred_probabilities = [0 for i in range(self.num_states)]
        for i in range(0, self.num_states):
            for j in range(3):
                if (i-1 >= 0):
                    pred_probabilities[i] += self.control_model[control][2-j]*curr_probabilities[(i-1+j)%self.num_states]
                else:
                    pred_probabilities[i] += self.control_model[control][2-j]*curr_probabilities[i-1+j]

        # === STATE UPDATE ===
        for i in range(self.num_states):
            color_index = self.colors.index(measurement)
            pred_probabilities[i] *= self.measurement_model[self.topological_map[i]][color_index]

        # Normalization
        total = 0.0
        for i in pred_probabilities:
            total += i
        
        for i in range(len(pred_probabilities)):
            pred_probabilities[i] = pred_probabilities[i] / total
        self.probabilities_location += [pred_probabilities]

        return True

    def plot(self, output_graph = True):
        print("========== BAYESIAN LOCALIZATION ==========")
        count = 1
        plot_index = range(self.num_states)
        for i in self.probabilities_location:
            print("step ", count, " : ", i.index(max(i)), max(i))
            count += 1
            if (output_graph):
                plt.bar(plot_index, i)
                plt.show()
        return True

    def measurement_callback(self, msg):
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.measured_rgb = np.array([r,g,b])
        
    def line_callback(self, data):
        index = int(data.data)
        self.line_idx = index	    

    # Ultimate gain 0.004
    def p(self):
	twist=Twist()
	twist.linear.x=0.03
	twist.angular.z=0.0
	kp = 0.001
	loop_rate = 20
	r=rospy.Rate(loop_rate)
	color_index = ["black", "green", "orange", "blue", "yellow"]
	paper_length = 0.20
	color_counter = 0
	straight_counter = 0
	correction = 0
	debug = False
	factor_of_safety = 1.45
	line_counter = 1
	update_once = True
	while(True):
		color, euclid_dist = self.rgb_diff()
		twist.linear.x=0.03
		if (color == 0 and straight_counter == 0 and line_counter != 0):
			diff = 320.0 - self.line_idx 
			correction = kp*diff
			twist.angular.z= correction 
		    	self.cmd_pub.publish(twist)
			debug = True
		elif(line_counter <= 20):
			line_counter += 1
			diff = 320.0 - self.line_idx 
			correction = kp*diff
			twist.angular.z= correction 
		    	self.cmd_pub.publish(twist)
			debug = True
		else:
			if (update_once):
				
				self.update(1, color_index[color])
				update_once = False
				for i in range(len(self.probabilities_location)):
					print(max(self.probabilities_location[i]), self.probabilities_location[i].index(max(self.probabilities_location[i])))
						#print(self.probabilities_location)
			if (straight_counter <= int(loop_rate * (paper_length / twist.linear.x))*factor_of_safety):
				straight_counter += 1
			else:
				straight_counter = 0
				line_counter = 0
				update_once = True
			
			twist.angular.z=0.0
			self.cmd_pub.publish(twist)
			debug = False

		print(correction, color_index[color], euclid_dist, debug)
		r.sleep() 
    	pass

    def pid(self):
	    twist=Twist()
	    twist.linear.x=0.05
	    twist.angular.z=0.0
	    #kp = 0.00936
	    #ki = 0.01783
	    #kd = 0.0012285
	    kp = 0.00936
	    ki = 0.0182
	    kd = 0.0011
	    icap = 0.01/ ki

	    rate = 20.0
	    r=rospy.Rate(rate)
	    prev = 320.0 - self.line_idx 
	    integral = 0
	    color_index = ["black", "green", "orange", "blue", "yellow"]
	    black_counter = 0
	    prev_color = 0
	    while(True):
		color, euclid_dist = self.rgb_diff()
		color_val = color_index[color]
		print(color_val)

		if (color_val == 0):
			black_counter += 1
		else:
			black_counter = 0

		if (black_counter >= 5):
			color = 0
		else:
			color = None

		if (color == 0):
			print(self.line_idx)
			diff = 320.0 - self.line_idx 
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
		    	self.cmd_pub.publish(twist)
			print("kp: ", kp*diff, "ki: ", ki*integral,"kd: ", kd*derivative)
		else:
			twist.angular.z=0.0
			self.cmd_pub.publish(twist)
		r.sleep() 
	    pass

    def rgb_diff(self):
	blue = [30.0, 140.0, 235.0]
	yellow = [252.0, 226.0, 0.0]
	orange = [252.0, 90.0, 20.0] # g -> 110-60, b -> 2-60 (actually orange -> faded orange)
	green = [208.0, 240.0, 1.0]
	black = [214.0, 180.0, 184.0]
	blue_dist = self.euclidean_distance(blue)
	yellow_dist = self.euclidean_distance(yellow)
	green_dist = self.euclidean_distance(green)
	orange_dist = self.euclidean_distance(orange)
	black_dist = self.euclidean_distance(black)
	distances = [black_dist, green_dist, orange_dist, blue_dist, yellow_dist]
	return distances.index(min(distances)), min(distances)

    def euclidean_distance(self, vector):
	return math.sqrt((vector[0]-self.measured_rgb[0])**2 + (vector[1]-self.measured_rgb[1])**2 + (vector[0]-self.measured_rgb[0])**2)


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
      
    color_map = [0,1,2,3] ### A sample map with 4 colours in a row
                 
    rospy.init_node('bayes_loc')
    BL=BayesLoc(color_map)
    rospy.sleep(0.5)
    
    BL.p()

    try:
        while (1):
            key = getKey()
            if (key == '\x03'): #1.22:bayesian.curPos >= 1.6 or
                rospy.loginfo('Finished!')
                break
            
            rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
            rospy.loginfo("Line index: {}".format(BL.line_idx))
                
#    except Exception as e:
#        print("comm failed:{}".format(e))

    finally:

            ### Stop the robot when code ends
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_publisher.publish(twist)





