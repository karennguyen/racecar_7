#!/usr/bin/python

#uses potential field theory to have the racecar explore the world it's in

#import statements
import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import * 
from ackermann_msgs.msg import AckermannDriveStamped

RADIANS_PER_TICK = math.pi  / 720

class Explorer():
	
	def __init__(self):
		#constants for racecar speed and angle calculations
		self.pSpeed = 2 #tweak
		self.pAngle = .7
		#positive charge behind racecar to give it a "kick" (forward vector)
		self.propelling_charge = 5
		#more constants
		self.charge = 0.01
	  	self.safety_threshold = 0.3
		self.speeds = [1] #Creates a list of speeds

		self.stuck_time = 0
		self.stuck = False
		self.stuck_threshold = 2
		self.e1=0
		rospy.init_node("explorer")
		
		self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)
		self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped)
		rospy.on_shutdown(self.shutdown)
        
	'''
	Get steering command as a tuple of (steering_angle, speed)
	L is the laser scan data
	'''
	def get_driving_info(self, L):
      
		#variables that will be used to calculate sum of vectors 
		force_x = 0  
		force_y = 0 
        
		for i in range(len(L)):
      		#actually finds sum of x and y components in vectors
			force_x += self.charge /(L[i]) * math.sin(i * RADIANS_PER_TICK)
			force_y += self.charge /(L[i]) * math.cos(i * RADIANS_PER_TICK)
        
		#shifts vector to go in the right direction (from behind racecar to front)
		if (self.stuck):
			force_x = -self.propelling_charge - force_x
		else:
			force_x = self.propelling_charge - force_x
		                                  
		speed = self.pSpeed * math.sqrt(force_x ** 2 + force_y ** 2) * np.sign(force_x)
		angle = (self.pAngle * math.atan2(-force_y, force_x) * np.sign(force_x)) * -1
                error = force_y
                Kd=-1.2
                angle+=Kd*(self.e1-error) 
                self.e1=error                             
  		return (speed, angle)
                          
	def laser_callback(self, scan):
		cmd = self.get_driving_info(scan.ranges[180:900]) #returns a tuple with the speed and angle 
		speed = cmd[0] #information given by the method
		angle = cmd[1]
        
		if (min(scan.ranges[525:555]) < self.safety_threshold):
		  	print "Robot suicide RIP"
			speed = -0.2
		elif (self.is_stuck()):
			#unstuck
			#speed = -0.7
			self.stuck_time = rospy.get_time()
			self.stuck = True
		elif (self.stuck): #if it is stuck and the time has not elapsed
			print 'Is Stuck'
			if not rospy.get_time() - self.stuck_time < self.stuck_threshold:
		    		self.stuck = False
		  
		self.speeds.append(speed)

		if(len(self.speeds) > 40):
			self.speeds.pop(0)
		  
		msg = AckermannDriveStamped()
		msg.drive.speed = max(-.5,speed)
		msg.drive.steering_angle = angle
		self.pub.publish(msg)        
      
	'''
	Define List of speeds
	if the speed has been static the entire duration of the list, return true
	'''
	def is_stuck(self):
		for i in self.speeds:
			if abs(i) > .3:
				return False
		return True
        
	def shutdown(self):
		self.pub.publish(AckermannDriveStamped())
		rospy.sleep(1)
      
if __name__ == '__main__':
	Explorer()
	rospy.spin()
