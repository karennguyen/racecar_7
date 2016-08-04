#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy as np
# node specific imports
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs

RADIANS_PER_TICK = math.pi / 720
MAX_SPEED = 5.0

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
      
        #use this for small step accerleration
        self.last_speed=0
        self.e1 = 0
	self.e2 = 0
        '''
        Node setup and start
        '''
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        rospy.Subscriber('scan', LaserScan, self.laserCall)
        rospy.Subscriber('/color', String, self.blobCall)
        
        '''
        Leave the robot going until roscore dies, then set speed to 0
        '''
        self.drive.publish(AckermannDriveStamped())
        print ("init")
    '''
    ******************************************************************************************
    *                                  Begin Driving stuffs                                  *
    ******************************************************************************************
    '''
    
    '''
    Get the error between the car and the wall for wall following
    '''
    def getError(self, goal, L, begin, end):
        p = sorted(L[begin:end])
        return p[5]-goal
    '''
    Get the steering command for the wall follower
    '''
    def getWallFollowCommand(self, error):
        Kp = .7
        Kd = 0.6
        de = error-self.e2
        self.e2 = self.e1
        self.e1 = error

        u=Kp*error+Kd*de
        return u
        
   
    '''
    callback for the laser subscriber
    does the drivin stuffs
    '''
    def laserCall(self,msg):
        
        #create the new message
        drive_cmd = AckermannDriveStamped()
        
        #get the color of the blob if there is a recent one (or receive a snarky comment)
        blob = self.recentBlob()
        print blob
        #red blob detected, follow right, into the roundabout
        #if blob == "red":
        #chill out around that turn
        speed = MAX_SPEED
        error=self.getError(0.6, msg.ranges, 80, 460)
        angle=-self.getWallFollowCommand(error)
            
        #Use the potential field to set speed and angle
        #else: 
            #speed,angle= self.get_driving_info(msg.ranges[180:900])
    	rospy.loginfo(error)
        '''
        Small step acceleration
        '''
	'''
        #if the sign is the same , the change is large, and the car is near stopped, increase slowly
        if(abs(speed - self.last_speed)>.2 and abs(self.last_speed)<.5 and np.sign(speed)==np.sign(self.last_speed)):
            speed = self.last_speed + .5
        self.last_speed=speed
	print ("speed: " , speed , " angle: " , angle)
        '''
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
            if(rospy.get_time() - self.lastDetection < 10):
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
        self.blob_color = msg.data

if __name__=="__main__":
    rospy.init_node('time_trial_driver', anonymous=False)
    node = Follower()
    rospy.spin()
