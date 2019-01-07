#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
import imutils
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from cv_pipeline.msg import FloatList
from collections import deque

class base_movement:

  def __init__(self):
	#subscriber to topic from cv_camera
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber('/cv_camera/image_raw',Image,self.image_callback)
	self.info_sub = rospy.Subscriber("/cv_camera/camera_info", CameraInfo, self.info_callback)
	#publisher
	self.image_pub = rospy.Publisher('/base_movement/image',Image,queue_size=1)
	self.camera_model = None
	self.rate = rospy.Rate(20) #go through the loop 1 time every second
	self.realpts = deque(maxlen=64)
	self.pts = deque(maxlen=64) # pointer
    	self.counter = 0
    	self.dX = 0
	self.dY = 0
	self.rdX = 0
	self.rdY = 0
	self.direction = ""

  def info_callback(self, data):
        # Get a camera model object using image_geometry and the camera_info topic
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
	self.info_sub.unregister() #Only subscribe once

  def image_callback(self,msg):
	#convert ros message to opencv format	
	try:	
	  cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding=msg.encoding)
	except CvBridgeError as e:
	  print(e)

    	# define the lower and upper boundaries of the "blue"
    	# ball in the HSV color space
    	lower_blue = np.array([110,100,50]) 
    	upper_blue = np.array([130,255,255])
	frame = imutils.resize(cv_image, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	invert = cv2.bitwise_not(mask)
	# Otsu's thresholding after Gaussian filtering
	blur = cv2.GaussianBlur(invert,(5,5),0)
	ret, thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
	# noise removal
	kernel = np.ones((3,3),np.uint8)
	opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
	closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations = 2)
	cnts = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		rect = cv2.minAreaRect(c)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		self.rate.sleep()
		cv2.drawContours(frame, [box], 0,(0, 255, 255), 2)
		cv2.circle(frame, center, 5, (0, 0, 255), -1)
		self.pts.appendleft(center)
		self.realpts.appendleft(self.camera_model.projectPixelTo3dRay(center))

	for i in xrange(1, len(self.pts)):
		if self.pts[i - 1] is None or self.pts[i] is None:
			continue

		if self.counter >= 10 and i == 1 and self.pts[-10] is not None:
		# compute the difference between the x and y
		# coordinates and re-initialize the direction
		# text variables
			self.dX = self.pts[-10][0] - self.pts[i][0]
			self.dY = self.pts[-10][1] - self.pts[i][1]
			(dirX, dirY) = ("", "")
 
			# ensure there is significant movement in the x-direction
			if np.abs(self.dX) > 20: # more than 20 pixel btw x-Coordinates
				self.rdX = self.realpts[-10][0] - self.realpts[i][0]
				rospy.loginfo(self.rdX)
				dirX = "Right" if np.sign(self.dX) == 1 else "Left"
 
			# ensure there is significant movement in the y-direction
			if np.abs(self.dY) > 20:
				self.rdY = self.realpts[-10][1] - self.realpts[i][1]
				#rospy.loginfo(self.rdY)
				dirY = "Up" if np.sign(self.dY) == 1 else "Down"
 
			# handle when both directions are non-empty
			if dirX != "" and dirY != "":
				self.direction = "{}-{}".format(dirY, dirX)
 
			# otherwise, only one direction is non-empty
			else:
				self.direction = dirX if dirX != "" else dirY

		thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
		cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

	cv2.putText(frame, self.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,0.65, (0, 0, 255), 3)
	cv2.putText(frame, "dx: {}, dy: {}".format(self.dX, self.dY), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
	
	self.counter += 1
	cv2.waitKey(1)

	#convert opencv format back to ros format and publish result
	try:
	  self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))
	except CvBridgeError as e:
	  print(e)

if __name__ == '__main__':
	rospy.init_node('base_movement', anonymous=True)
	while not rospy.is_shutdown():
        	basemovement = base_movement()
        	rospy.spin()

