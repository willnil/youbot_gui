#!/usr/bin/env python

import rospy
import sys
import os
from cv_pipeline.msg import FloatList
from std_msgs.msg import Empty

# Initialize the node
rospy.init_node('pub_values')

# Create Publishers
arm_pub = rospy.Publisher('movement/arm_value', FloatList, queue_size=1)
base_pub = rospy.Publisher('movement/base_value', FloatList, queue_size=1)
motor_pub = rospy.Publisher('toggle_motor', Empty, queue_size=1)

# Create a rate
rate = rospy.Rate(0.25) # 1 message per 4 second

# Printing space limit [mm]
lim = [(0, 0), (-50, 50), (0, 394)]

# To be printed pose
filename = 'values.txt'
directory = '/home/user'
fullpath = os.path.join(directory, filename)

values = []

with open(fullpath, 'r') as f:
	for line in f:
		x, y, z = line.split(",")
		values.append((float(x), float(y), float(z)))

rospy.loginfo(values)

offset = (0, 0, 0)
movebasecounter = 0
movebasetoggle = 0
i = 0

# Run code in a loop until node is shutdown
while not rospy.is_shutdown():
	while i < len(values):
		if movebasetoggle == 1 and movebase_values[1] != 0:
			i -= 1
		print "Number of pose: {}".format(i)
		movebase_values = []
		if movebasecounter > 0: 
			values[i] = tuple(map(lambda x, y: x - y, values[i], offset))
		for j in range(3):
			if (values[i][j] > 0 and values[i][j] > lim[j][1]):
				movebase = values[i][j] - lim[j][1]
				movebasetoggle = 1
			elif (values[i][j] < 0 and values[i][j] < lim[j][0]):
				movebase = values[i][j] - lim[j][0]
				movebasetoggle = 1
			else:
				movebase = 0
			movebase_values.append(movebase)
		if i == 2:
			motor_pub.publish(Empty()) # 3D Pen Motor ON 
			print "------------3D Pen is switched ON-------------"
		if (movebase_values == [0,0,0]):
		      	print "------------Moving arms to the input pose--------------"
			arm_pub.publish(values[i])   # publish
		      	movebasetoggle = 0
		else:
			movebasecounter = 1
			print "------------Moving base to the new position--------------"
		      	base_pub.publish(tuple((movebase_values[0], movebase_values[1])))   # publish
		      	offset = tuple(map(lambda x, y: x + y, offset, movebase_values))
		i += 1
		if i == len(values)+1:
			motor_pub.publish(Empty()) # 3D Pen Motor OFF
			print "------------3D Pen OFF-------------"
		rate.sleep()
