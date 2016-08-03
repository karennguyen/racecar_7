#!/usr/bin/env python

# general imports for all python nodes
#WALL FOLLOW IMPLEMENTATION
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
        self.propelling_charge = 4
        #charge pushing on the car from the laser points
        self.charge = 0.005
        #use this for small step accerleration
        self.last_speed=0
        
        '''
        Node setup and start
        '''
        rospy.init_node('time_trial_driver', anonymous=False)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        rospy.Subscriber('scan', LaserScan, self.laserCall)
        rospy.Subscriber('/color', String, self.blobCall)
        
        '''
        Leave the robot going until roscore dies, then set speed to 0
        '''
        rospy.spin()
        self.drive.publish(AckermannDriveStamped())
    
    '''
    ******************************************************************************************
    *                                  Begin Driving stuffs                                  *
    ******************************************************************************************
    '''
    
    '''
    Get the error between the car and the wall for wall following
    '''
    def getError(self, goal, L, begin, end):
        return (min(L[begin:end])) - goal
        
    '''
    Get the steering command for the wall follower
    '''
    def getWallFollowCommand(self, error):
        Kp = .6
        Kd = .7
        de = error-self.e2
        self.e2 = self.e1
        self.e1 = error

        u=Kp*error+Kd*de
        return u
        
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
        if (self.stuck):
            force_x = -self.propelling_charge - force_x
        else:
            force_x = self.propelling_charge - force_x
        
        speed = self.pSpeed * math.sqrt(force_x ** 2 + force_y ** 2) * np.sign(force_x)
        angle = (self.pAngle * math.atan2(-force_y, force_x) * np.sign(force_x))*-1 # "Makes the equations easier" - Russ Tedrake
        
        
        return (speed, angle)
        
    '''
    callback for the laser subscriber
    does the drivin stuffs
    '''
    def laserCall(self,msg):
        
        #create the new message
        drive_cmd = AckermannDriveStamped()
        
        #get the color of the blob if there is a recent one (or receive a snarky comment)
        blob = self.recentBlob()
        
        #red blob detected, follow right, into the roundabout
        if blob == "red":
            
            #chill out around that turn
            speed = 1
            error=self.getError(0.5, msg.ranges, 200, 550)
            angle=self.getWallFollowCommand(error)
            
        #green blob detected, follow left, into the shortcut
        elif blob == "green": 
            
            #zip through that shortcut
            speed=MAX_SPEED
            error=self.getError(0.5, msg.ranges, 530,900)
            angle=self.getWallFollowCommand(error)
            
        #Use the potential field to set speed and angle
        else: 
            speed,angle= self.get_driving_info(msg.ranges[180:900])
    
        '''
        Small step acceleration
        '''
        #if the sign is the same , the change is large, and the car is near stopped, increase slowly
        if(abs(speed - self.last_speed)>.2 and abs(self.last_speed)<.5 and np.sign(speed)==np.sign(self.last_speed)):
            speed = self.last_speed + .5
        self.last_speed=speed
        
        #Assign speed and angle to the message
        drive_cmd.drive.steering_angle=angle
        drive_cmd.drive.speed=speed
        
        self.drive.publish(drive_cmd) # post this message
    
    
    '''
    *************************************************************************************************
    *                                  Blob Detection and response                                  *
    *************************************************************************************************
    '''
        
    '''
    Determines if a blob was recently found and returns the color if so
    '''
    def recentBlob(self):
        try:
            if(rospy.time() - self.lastDetection < 1):
                return self.blob_color
            else:
                return "lol jk fam"
        except Exception:
            return "Wow, you made quite the error dummy"
        
    '''
    Blob detector callback
    Sets the time and color of the blob detected
    '''
    def blobCall(self, msg):
        self.lastDetection = rospy.get_time()
        self.blob_color = msg

if __name__=="__main__":
    Follower()
