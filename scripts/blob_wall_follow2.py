#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs
from bwsi_race.msg import blob_detect

class Follower():
    e1 = 0
    e2 = 0
    right = True
    death = False
    wall = False

    def getError(self, goal, L, begin, end):
        return (min(L[begin:end])) - goal

    #get the angle
    def getSteeringCmd(self, error, fullLeft, fullRight):
        Kp = .6
        Kd = .7
        de = error - self.e2
        self.e2 = self.e1
        self.e1 = error

        u = Kp * error + Kd * de
        return u

    def switch(self, color):
        wall = True
        if(color == "red"):
            self.right = True
        else:
            self.right = False

    #passed to the subscriber
    def laserCall(self,msg):
        if(not self.wall):
            return
        # fill out fields in ackermann steering message (to go straight)
        drive_cmd = AckermannDriveStamped()

        if not self.right: #right
            error = self.getError(0.5, msg.ranges, 200, 550)
            if(error > -.5):
                angle = self.getSteeringCmd(error, -1, 1)
            else:
                angle = 1
        else: #left
            error = self.getError(0.5, msg.ranges, 530,900)
            if(error > -.5):
                angle = self.getSteeringCmd(error, -1, 1)
            else:
                angle = -1

        self.death = min(msg.ranges[520:560])<.2
        drive_cmd.drive.steering_angle = angle

        if self.death:
            print "Wall follower dead"
            drive_cmd.drive.speed = -.1
        else:
            print "Angle is %f" % angle
            drive_cmd.drive.speed = 1

        self.drive.publish(drive_cmd) # post this message

    def blobCall(self,msg):
        '''
        msg.color    -> String
        msg.height   -> float
        msg.location -> float
        '''
        if(self.wall):
            return
	print ("Height: ", msg.height.data , "Location: " ,msg.location.data)
        if (msg.height.data > 200):
	    print ("In box")
            self.switch(msg.color)
            return

        drive_cmd = AckermannDriveStamped()

        drive_cmd.drive.steering_angle =.5 - msg.location.data
        drive_cmd.drive.speed = 1
        self.drive.publish(drive_cmd)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        rospy.sleep(1)

    def __init__(self,bool_direction):
        print "Beginning wall follow"
        #setup the node
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.right = bool_direction

        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)

        #sets the subscriber
        rospy.Subscriber('scan', LaserScan, self.laserCall)
        rospy.Subscriber('blob_info', blob_detect,self.blobCall)
        rospy.spin()
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())

if __name__=="__main__":
	Follower(True)
