#!/usr/bin/env python

from std_msgs.msg import *
import rospy as rsp
import cv2
import numpy as np

class ColorPub():

    def init(self):

        self.bridge = CvBridge()

        self.zed_pub = rsp.Publisher("/image_echo", Image, queue_size = 1)
        self.color_pub = rsp.Publisher("/color", String, queue_size = 1)

        self.zed_img = rsp.Subscriber("/camera/rgb/image_rect_color", Image, self.callback)

        rsp.init_node("color_node")

    def callback(self, img):
        img = self.bridge.imgmsg_to_cv2(img)

        color = self.detectColor(img)
        self.color_pub.publish(color)

    def detectColor():
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

        #red stuff
    	hue_red_min = 0
    	hue_red_max = 30

    	sat_red_min = .666
    	sat_red_max = 1

    	val_red_min = .79
    	val_red_max = 1

     	maskRed = cv2.inRange(hsv, np.array([hue_red_min / 2, int(sat_red_min * 255), int(val_red_min * 255)]), np.array([hue_red_max / 2, int(sat_red_max * 255), int(val_red_max * 255)]))

    	contours_red, hierarchy_red = cv2.findContours(maskRed1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours_green) == 0:
            contRedArea = [ (cv2.contourArea(c), (c) ) for c in contours_red]
            contRedArea = sorted(contRedArea, reverse=True, key=lambda x: x[0])
            officCont = contRedArea[0][1]
            blobD.color = "red"
        elif len(contours_red) == 0:
            contGreenArea = [ (cv2.contourArea(c), (c) ) for c in contours_green]
            contGreenArea = sorted(contGreenArea, reverse=True, key=lambda x: x[0])
            officCont = contGreenArea[0][1]
            blobD.color = "green"
        else:
            contGreenArea = [ (cv2.contourArea(c), (c) ) for c in contours_green]
            contGreenArea = sorted(contGreenArea, reverse=True, key=lambda x: x[0])
            contRedArea = [ (cv2.contourArea(c), (c) ) for c in contours_red]
            contRedArea = sorted(contRedArea, reverse=True, key=lambda x: x[0])
        if (max(contGreenArea[0][0], contRedArea[0][0]) == contGreenArea[0][0]):
            officCont = contGreenArea[0][1]
            blobD.color = "green"
        else:
            officCont = contRedArea[0][1]
            blobD.color = "red"

        return blobD.color

if __name__ == "__main__":
    ColorPub()
    rsp.spin()
