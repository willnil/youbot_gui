#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class detect_colour:

  def __init__(self):
	#subscriber to topic from cv_camera
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber('/cv_camera/image_raw',Image,self.image_callback)
	#publisher  
	self.image_pub = rospy.Publisher('/detect_color/image_blue',Image,queue_size=1)
	self.rate = rospy.Rate(1) #go through the loop 1 time every second

  def image_callback(self,msg):
	#convert ros message to opencv format	
	try:	
	  cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding=msg.encoding)
	except CvBridgeError as e:
	  print(e)
	#blue = self.detectcolour_segmentation(cv_image)
	#gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
	#canny edge detection 	
	#edges = cv2.Canny(gray,50,150,apertureSize = 3)
	#self.detectcorner(edges, cv_image)
	#self.surfdetector(gray, cv_image)
	#img2 = self.segmentation(cv_image)
	#img2 = self.orbdetector(gray, cv_image)
	img2 = self.findContours(cv_image)
	#squares = self.find_squares(cv_image)
	#cv2.drawContours( cv_image, squares, -1, (0, 255, 0), 3 )
        cv2.waitKey(2)
	#convert opencv format back to ros format and publish result
	try:
	  self.image_pub.publish(self.bridge.cv2_to_imgmsg(img2,"bgr8"))
	except CvBridgeError as e:
	  print(e)

  def detectcolour_segmentation(self, image):
	# Converts images from BGR to HSV 
    	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower_blue = np.array([110,100,50]) 
    	upper_blue = np.array([130,255,255])

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	output = cv2.bitwise_and(image, image, mask = mask)
	blur = cv2.GaussianBlur(output, (7, 7), 0)
	cv2.waitKey(0)
 	return blur

  def detectcolour_contours(self, image):
	# Converts images from BGR to HSV 
    	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower_blue = np.array([110,100,50]) 
    	upper_blue = np.array([130,255,255])

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	invert = cv2.bitwise_not(mask)
	cv2.waitKey(0)
 	return invert

  def segmentation(self, cv_image):
	blue = self.detectcolour_segmentation(cv_image)
	gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
	# Otsu's thresholding after Gaussian filtering
	blur = cv2.GaussianBlur(gray,(5,5),0)
	ret, thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
	# noise removal
	kernel = np.ones((3,3),np.uint8)
	opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 5)
	# sure background area
	sure_bg = cv2.dilate(opening,kernel,iterations=3)
	# Finding sure foreground area
	dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
	ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
	# Finding unknown region
	sure_fg = np.uint8(sure_fg)
	unknown = cv2.subtract(sure_bg,sure_fg)
	# Marker labelling
	ret, markers = cv2.connectedComponents(sure_fg)
	# Add one to all labels so that sure background is not 0, but 1
	markers = markers+1
	# Now, mark the region of unknown with zero
	markers[unknown==255] = 0
	markers = cv2.watershed(blue,markers)
	# boundary region 
	cv_image[markers == -1] = [255,0,0]
	# find contours
	img2 = cv_image.copy()
	markers1 = markers.astype(np.uint8)
	ret, m2 = cv2.threshold(markers1, 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
	_, contours, hierarchy = cv2.findContours(m2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)    
	for c in contours:
		cv2.drawContours(img2, c, -1, (0, 255, 0), 2)
	return img2

  def findContours(self, cv_image):
	blue = self.detectcolour_contours(cv_image)
	# Otsu's thresholding after Gaussian filtering
	blur = cv2.GaussianBlur(blue,(5,5),0)
	ret, thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
	# noise removal
	kernel = np.ones((3,3),np.uint8)
	opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
	closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations = 2)
	_, contours, hierarchy = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	img2 = cv_image.copy()
	for c in contours:
		cv2.drawContours(img2, c, -1, (0, 255, 0), 2)
	return img2

  def find_squares(self, img):
	img = self.detectcolour(img)
    	squares = []
    	for gray in cv2.split(img):
        	for thrs in xrange(0, 255, 26):
            		if thrs == 0:
                		bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                		bin = cv2.dilate(bin, None)
            		else:
                		_retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            		bin, contours, _hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            		for cnt in contours:
                		cnt_len = cv2.arcLength(cnt, True)
                		cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                		if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):
                    			cnt = cnt.reshape(-1, 2)
                    			max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    			if max_cos < 0.1:
                        			squares.append(cnt)
	return squares

  def angle_cos(self, p0, p1, p2):
    	d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    	return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

  def detectcorner(self, image, cv_image):
	# detect corners with the goodFeaturesToTrack function. 
	corners = cv2.goodFeaturesToTrack(image, 27, 0.01, 10)
	if corners is None: return 
	corners = np.int0(corners) 
	
	# we iterate through each corner,  
	# making a circle at each point that we think is a corner. 
	for i in corners: 
    		x, y = i.ravel() 
    		cv2.circle(cv_image, (x, y), 3, 255, -1)

  def surfdetector(self, image, cv_image):
	# Create SURF object. You can specify params here or later.
	# Here I set Hessian Threshold to 400
	surf = cv2.xfeatures2d.SURF_create(400)
	# Find keypoints and descriptors directly
	kp, des = surf.detectAndCompute(image,None)
	#img1 = cv2.drawKeypoints(image,kp,None)
	
	pts = [p.pt for p in kp]
	for point in pts:
		point = np.round(point).astype("int") #conversion float to int
		cv2.circle(cv_image, tuple(point), 3, 255, -1)

  def orbdetector(self, image, cv_image):
	# Initiate STAR detector
	orb = cv2.ORB_create()
	# find the keypoints and compute the descriptors with ORB
	kp, des = orb.detectAndCompute(image, None)
	# draw only keypoints location,not size and orientation
	img2 = cv2.drawKeypoints(cv_image,kp,None,color=(0,255,0), flags=0)
	return img2

if __name__ == '__main__':
	#create ROS node	
	rospy.init_node("hough_line_transform")
	while not rospy.is_shutdown():
        	detect = detect_colour()
        	rospy.spin()
        
