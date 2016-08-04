#!/usr/bin/env python
import rospy as rsp
from sensor_msgs.msg import Image
from std_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from bwsi_race.msg import img_info
import cv2
import numpy as np
import math

class BlobDetection:

	def __init__(self):
		self.bridge = CvBridge() #allows us to convert our image to cv2

		self.zed_pub = rsp.Publisher("/image_echo", Image, queue_size = 1)
		self.imginfo_pub = rsp.Publisher("/exploring_challenge", img_info, queue_size = 1)

		self.zed_img = rsp.Subscriber("/camera/rgb/image_rect_color", Image, self.detect_img) #subscribes to the ZED camera image
		self.odom_sub = rsp.Subscriber("/vesc/odom", Odometry, self.odom_callback)

		self.last_arb_position = Point()
		self.gone_far_enough = True

		self.header = std_msgs.msg.Header()
		self.heightThresh = 75 #unit pixels MUST TWEAK
		self.odomThresh = 1 #unit meters
		self.blob_msg = img_info()

		rsp.init_node("vision_node")
    
	def odom_callback(self, odom): #odom callback
		dist = math.sqrt((self.last_arb_position.x - odom.pose.pose.position.x)**2 + (self.last_arb_position.y - odom.pose.pose.position.y)**2)                               
		if(dist > 1):#if moved a meter since last
			self.gone_far_enough = True               
			self.last_arb_position.x = odom.pose.pose.position.x
			self.last_arb_position.y = odom.pose.pose.position.y
      		else:
        		self.gone_far_enough = False

	def detect_img(self, img): #image callback
		if(not self.gone_far_enough):
			return
		self.blob_msg.header = self.header

		img_data = self.bridge.imgmsg_to_cv2(img) #changing image to cv2

		processed_img_cv2 = self.process_img(img_data) #passing image to process_img function
		processed_img = self.bridge.cv2_to_imgmsg(processed_img_cv2, "bgr8") #convert image back to regular format (.png?)
		cv2.imwrite("/home/racecar/challenge_photos/%i.png" % rsp.get_time(), processed_img_cv2)
		self.blob_msg.img_file = processed_img

		self.imginfo_pub.publish(self.blob_msg)
		self.zed_pub.publish(processed_img)

	def process_img(self, img):
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #converting to HSV
		
		#GREEN
		hue_green_min = 100
		hue_green_max = 154

		sat_green_min = .4
		sat_green_max = 1

		val_green_min = .15
		val_green_max = 1

		green_bounds = np.array([hue_green_min / 2, int(sat_green_min * 255), int(val_green_min * 255)]), np.array([hue_green_max / 2, int(sat_green_max * 255), int(val_green_max * 255)])
		
		maskGreen = cv2.inRange(hsv, green_bounds[0], green_bounds[1])
		contours_green, hierarchy_green = cv2.findContours(maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		#RED
		hue_red_min = 0
		hue_red_max = 30

		sat_red_min = .5
		sat_red_max = 1

		val_red_min = 0.3
		val_red_max = 1
		
		red_bounds = np.array([hue_red_min / 2, int(sat_red_min * 255), int(val_red_min * 255)]), np.array([hue_red_max / 2, int(sat_red_max * 255), int(val_red_max * 255)])
		#red_bounds = np.array([0,190,200]), np.array([15, 255, 255])
	 	maskRed = cv2.inRange(hsv, red_bounds[0], red_bounds[1])
		contours_red, hierarchy_red = cv2.findContours(maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		#YELLOW
		hue_yellow_min = 40
		hue_yellow_max = 90

		sat_yellow_min = 0.4
		sat_yellow_max = 1

		val_yellow_min = 0.25
		val_yellow_max = 1
                                       
		yellow_bounds = np.array([hue_yellow_min / 2, int(sat_yellow_min * 255), int(val_yellow_min * 255)]), np.array([hue_yellow_max / 2, int(sat_yellow_max * 255), int(val_yellow_max * 255)])
                                       
	 	maskYellow = cv2.inRange(hsv, yellow_bounds[0], yellow_bounds[1])
		contours_yellow, hierarchy_yellow = cv2.findContours(maskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        	#BLUE
		hue_blue_min = 200
		hue_blue_max = 240

		sat_blue_min = 0.58
		sat_blue_max = .75

		val_blue_min = .275
		val_blue_max = .43
                                       
		blue_bounds = np.array([hue_blue_min / 2, int(sat_blue_min * 255), int(val_blue_min * 255)]), np.array([hue_blue_max / 2, int(sat_blue_max * 255), int(val_blue_max * 255)])
		
	 	maskBlue = cv2.inRange(hsv, blue_bounds[0], blue_bounds[1])
		contours_blue, hierarchy_blue = cv2.findContours(maskBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    		contour_list = [contours_red, contours_green, contours_yellow, contours_blue]
    		string_list = ["red", "green", "yellow", "blue"]
                             
		try:
			for i in range(len(contour_list)):
		        	if len(contour_list[i]) != 0:
				  	contArea = [(cv2.contourArea(c), (c) ) for c in contour_list[i]]
				  	contArea = sorted(contArea, reverse = True, key = lambda x: x[0])
					cont = contArea[0][1]
				  	M = cv2.moments(cont)
		                  			
		          		x, y, w, h = cv2.boundingRect(cont)

				  	if  h > self.heightThresh: #comparing height of contour to height threshold param
						print (string_list[i] , "found")
				    		self.blob_msg.color = string_list[i] #setting the color field in the custom message type blob_msg
				    		self.blob_msg.shape = "other" # change??
				    		cv2.drawContours(img, cont, -1, (255, 255, 255), 10) 
		
						if M['m00'] != 0:
					        	cx = int(M['m10']/M['m00'])
					      		cy = int(M['m01']/M['m00'])
					      		center = (cx, cy)
					      		cv2.circle(img, center, 5, (60, 0, 0), -1)
					      		cv2.rectangle(img, (x, y), (x + w, y + h), (100, 50, 50), 2)
					      		font = cv2.FONT_HERSHEY_SIMPLEX
					      		cv2.putText(img, string_list[i], center, font, 1,(0,0,0) , 4)
		                
		except Exception, e:
			print str(e)
		
		return img

if __name__ == "__main__":
	node = BlobDetection()
	rsp.spin()
