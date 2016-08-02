#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy as np
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs


MAX_SPEED=2.0

class Follower():
    
    '''
    ******************************************************************************************
    *                                  Begin Driving stuffs                                  *
    ******************************************************************************************
    '''
    
    def findLargestSpace(self, L, threshold):
        center = 0
        largestSpace = 0     

        for i in range(len(L)):
            if(L[i]>threshold):
                count = 0
                while L[i]>threshold:
                    count += 1
                    i += 1
                    if(i>=len(L)):
                        break
                    if(count > largestSpace):
                        largestSpace = count
                        center = (count / 2) + i
        return center # a point out of 1081
        
        
    '''
    callback for the laser subscriber
    '''
    def laserCall(self,msg):
        
        #create the new message
        drive_cmd = AckermannDriveStamped()
        
        drive_cmd.drive.steering_angle=(540-self.findLargestSpace(msg.ranges, 2)) /200
        
        drive_cmd.drive.speed=MAX_SPEED
    
        self.drive.publish(drive_cmd) # post this message
      
    
    '''
    *************************************************************************************************
    *                      Constructor and initialization of instance variables                     *
    *************************************************************************************************
    '''
    def __init__(self):        
        
        '''
        Node setup and start
        '''
        rospy.init_node('grand_prix', anonymous=False)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        rospy.Subscriber('scan', LaserScan, self.laserCall)
        
        '''
        Leave the robot going until roscore dies, then set speed to 0
        '''
        rospy.spin()
        self.drive.publish(AckermannDriveStamped())
        
if __name__=="__main__":
    Follower(True)
