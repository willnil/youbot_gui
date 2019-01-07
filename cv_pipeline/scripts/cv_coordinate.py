#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class hough_line_transform:

  def __init__(self):
	#subscriber to topic from cv_camera
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber('/cv_camera/image_raw',Image,self.image_callback)
	self.info_sub = rospy.Subscriber("/cv_camera/camera_info", CameraInfo, self.info_callback)
	#publisher  
	self.image_pub = rospy.Publisher('/hough_line_transform/image_line',Image,queue_size=1)
	self.camera_model = None
	self.rate = rospy.Rate(1) #go through the loop 1 time every second

  def image_callback(self,msg):
	#convert ros message to opencv format	
	try:	
	  cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding=msg.encoding)
	except CvBridgeError as e:
	  print(e)
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
	#canny edge detection 	
	edges = cv2.Canny(gray,50,150,apertureSize = 3)
	#hough line transform
        lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength=100,maxLineGap=10)
        self.plotLines(cv_image, lines, (0,255,0))
        cv2.waitKey(2)
	#convert opencv format back to ros format and publish result
	try:
	  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
	except CvBridgeError as e:
	  print(e)

  def info_callback(self, data):
        # Get a camera model object using image_geometry and the camera_info topic
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
	self.info_sub.unregister() #Only subscribe once

  def convert_to_3dpt(self, pixelpt):
	x = int(pixelpt[0])
	y = int(pixelpt[1])
	v = self.camera_model.projectPixelTo3dRay((x, y))
	return v

  def plotLines(self, img, lines, color):
	if lines is None: return

	for line in lines:
            	x1,y1,x2,y2 = line[0]
            	cv2.line(img,(x1,y1),(x2,y2),color,2) #draw lines
		pixelpt = (x2, y2)
		coor = self.convert_to_3dpt(pixelpt)
		rospy.loginfo(coor)

if __name__ == '__main__':
	#create ROS node	
	rospy.init_node("hough_line_transform")
	while not rospy.is_shutdown():
        	houghline = hough_line_transform()
        	rospy.spin()
        
