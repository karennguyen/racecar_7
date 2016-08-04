#!/usr/bin/env python
import rospy as rsp
from sensor_msgs.msg import Image
from std_msgs.msg import *
from bwsi_race.msg import blob_detect
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ZedCamPub:

	def __init__(self):

		self.bridge = CvBridge()

		self.zed_pub = rsp.Publisher("/image_echo", Image, queue_size=1)
		self.loc_height_pub = rsp.Publisher("/blob_info", blob_detect, queue_size=1)
		
		self.zed_img = rsp.Subscriber("/camera/rgb/image_rect_color", Image, self.detect_img)
		
		self.header = std_msgs.msg.Header()
		
	def detect_img(self, img):
		img_data = self.bridge.imgmsg_to_cv2(img)

		processed_img_cv2 = self.process_img(img_data)
		processed_img = self.bridge.cv2_to_imgmsg(processed_img_cv2, "bgr8")

		self.zed_pub.publish(processed_img)

	def process_img(self, img):
		blobD = blob_detect()	

		hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
		
		#GREEN STUFF
		hue_green_min = 100
		hue_green_max = 154

		sat_green_min = .4
		sat_green_max = 1

		val_green_min = .4
		val_green_max = 1

		maskGreen = cv2.inRange(hsv, np.array([hue_green_min / 2, int(sat_green_min * 255),int(val_green_min * 255)]), np.array([hue_green_max / 2, int(sat_green_max * 255), int(val_green_max * 255)]))

		contours_green, hierarchy_green = cv2.findContours(maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		hue_red_min = 200#0
		hue_red_max = 260#30

		sat_red_min = .666
		sat_red_max = 1

		val_red_min = .79
		val_red_max = 1
		
	 	maskRed = cv2.inRange(hsv, np.array([hue_red_min / 2, int(sat_red_min * 255), int(val_red_min * 255)]), np.array([hue_red_max / 2, int(sat_red_max * 255), int(val_red_max * 255)]))

		contours_red, hierarchy_red = cv2.findContours(maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		try:		
			if len(contours_green) == 0:
				contRedArea = [ (cv2.contourArea(c), (c) ) for c in contours_red]
				contRedArea = sorted(contRedArea, reverse=True, key=lambda x: x[0])
				officCont = contRedArea[0][1]
				blobD.color.data = "red"
			elif len(contours_red) == 0:
				contGreenArea = [ (cv2.contourArea(c), (c) ) for c in contours_green]
				contGreenArea = sorted(contGreenArea, reverse=True, key=lambda x: x[0])
				officCont = contGreenArea[0][1]
				blobD.color.data = "green"
			else:
				contGreenArea = [ (cv2.contourArea(c), (c) ) for c in contours_green]
				contGreenArea = sorted(contGreenArea, reverse=True, key=lambda x: x[0])
				contRedArea = [ (cv2.contourArea(c), (c) ) for c in contours_red]
				contRedArea = sorted(contRedArea, reverse=True, key=lambda x: x[0])
				if( max(contGreenArea[0][0], contRedArea[0][0]) == contGreenArea[0][0]):
					officCont = contGreenArea[0][1]
					blobD.color.data = "green"
				else:
					officCont = contRedArea[0][1]
					blobD.color.data = "red"

			cv2.drawContours(img, officCont, -1, (120, 0, 0), 4)
			MGr = cv2.moments(officCont)
			if MGr['m00'] != 0:
				cx = int(MGr['m10']/MGr['m00'])
				cy = int(MGr['m01']/MGr['m00'])
				centergr = (cx, cy)
				cv2.circle(img, centergr, 7, (60, 0, 0), -1)
				#green rect	
				x, y, w, h = cv2.boundingRect(officCont)	
				cv2.rectangle(img, (x,y), (x+w, y+h), (100, 50, 50), 2)
				#location
				height = np.size(img, 0)
				width = np.size(img, 1)
							
				blobD.header = self.header
				blobD.height = Float64(float(h))
				blobD.location =  Float64(float(cx)/float(width))
				self.loc_height_pub.publish(blobD)
			else:
				#print "didn't get into the if"
				pass
			
		except Exception, e:
			#print str(e)
			pass
		return img

if __name__ == "__main__":
	node = ZedCamPub()
	rsp.init_node("zed_pub")
	rsp.spin()
