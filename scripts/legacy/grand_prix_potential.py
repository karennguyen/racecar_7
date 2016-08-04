#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy as np
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs

RADIANS_PER_TICK = math.pi / 720
MAX_SPEED = 2.0

class Follower():


    '''
    *************************************************************************************************
    *                      Constructor and initialization of instance variables                     *
    *************************************************************************************************
    '''
    def __init__(self):
        '''
        Instance variables
        '''
        #constants for racecar speed and angle calculations
        self.pSpeed = 0.3
        self.pAngle = 1
        #positive charge behind racecar to give it a "kick" (forward vector)
        self.propelling_charge = 6
        #charge pushing on the car from the laser points
        self.charge = 0.005
        
        
        '''
        Node setup and start
        '''
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        rospy.Subscriber('scan', LaserScan, self.laserCall)
        
        '''
        Leave the robot going until roscore dies, then set speed to 0
        '''
        self.drive.publish(AckermannDriveStamped())
    
    '''
    ******************************************************************************************
    *                                  Begin Driving stuffs                                  *
    ******************************************************************************************
    '''
    
    
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
            force_x += self.charge /(L[i] ** 2) * math.sin(i * RADIANS_PER_TICK)
            force_y += self.charge /(L[i] ** 2) * math.cos(i * RADIANS_PER_TICK)
        
        #shifts vector to go in the right direction (from behind racecar to front)                                          
        speed = self.pSpeed * math.sqrt(force_x ** 2 + force_y ** 2) * np.sign(force_x)
        angle = (self.pAngle * math.atan2(-force_y, force_x) * np.sign(force_x))* -1 #"Makes the equations easier" - Monday Seminar Dude
        
        return (speed, angle)
        
    '''
    callback for the laser subscriber
    '''
    def laserCall(self, msg):
        
        #create the new message
        drive_cmd = AckermannDriveStamped()
        cmd = self.get_driving_info(msg.ranges[180:900])
        speed = cmd[0]
	angle = cmd[1]
        
        #Assign speed and angle to the message
        drive_cmd.drive.steering_angle = angle
        drive_cmd.drive.speed = speed
    
        self.drive.publish(drive_cmd) # post this message
      
    
        
if __name__=="__main__":
    rospy.init_node('grand_prix', anonymous=False)
    Follower()
    rospy.spin()
