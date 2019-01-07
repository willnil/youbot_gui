#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
import imutils
import image_geometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
from collections import deque

def talker():
    rospy.init_node('base_movement', anonymous=True)
    pub = rospy.Publisher('coordinate', Float32, queue_size=1)
    rate = rospy.Rate(20) #10Hz

    # define the lower and upper boundaries of the "blue"
    # ball in the HSV color space
    lower_blue = np.array([110,100,50]) 
    upper_blue = np.array([130,255,255])

    pts = deque(maxlen=64) # pointer
    counter = 0
    (dX, dY) = (0, 0)

    camera = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
		(grabbed, frame) = camera.read()
		if not grabbed:
			break
		frame = imutils.resize(frame, width=600)
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
			#print(center)
			pub.publish(center)
			rospy.loginfo(center)
			rate.sleep()
			cv2.drawContours(frame, [box], 0,(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
		pts.appendleft(center)

		for i in xrange(1, len(pts)):
			if pts[i - 1] is None or pts[i] is None:
				continue

			if counter >= 10 and i == 1 and pts[-10] is not None:
			# compute the difference between the x and y
			# coordinates and re-initialize the direction
			# text variables
				dX = pts[-10][0] - pts[i][0]
				dY = pts[-10][1] - pts[i][1]

			thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
			cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
		
		cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
	
		cv2.imshow("Frame", frame)
		counter += 1
		if cv2.waitKey(1) & 0xFF==ord('q'):
			break

    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	video_capture.release()
	cv2.destroyAllWindows()
	pass

