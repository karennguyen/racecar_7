#!/usr/bin/env python
# general imports for all python nodes
import rospy
import math
import numpy

# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan, Joy # joystick and laser scanner msgs

scan = [0.0, 0.0]

class Accel():
    def __init__(self):
        global scan
        rospy.init_node('accel', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        rate = 10             # messages/sec (also determines latency)
        r = rospy.Rate(rate)

        # determine duration to run based on desired speed and distance
        speed = rospy.get_param('~speed', 2.0)       # meters/sec
        accel = rospy.get_param('~accel', 2.0)       # meters/sec^2
        delta_speed = accel / rate                   # maximum speed increment
        drive_time = 5.0                             # seconds

        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = 0.0
        last_speed = 0.0
        
        ticks = int(drive_time * rate) # convert drive time to ticks
        for t in range(ticks):
            limit_speed = speed
            if limit_speed - last_speed > delta_speed:
                limit_speed = last_speed + delta_speed
            drive_cmd.drive.speed = limit_speed
            drive_cmd.drive.steering_angle = steering_bias
            last_speed = limit_speed

            self.drive.publish(drive_cmd) # post this message
            r.sleep()                     # chill for a while

        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Accel()
    except:
        rospy.loginfo("Accel node terminated.")
