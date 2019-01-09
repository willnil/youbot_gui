#!/usr/bin/env python

import rospy
import sys
import os
from cv_pipeline.msg import FloatList

# Initialize the node
rospy.init_node('pub_values')

# Create Publishers
arm_pub = rospy.Publisher('movement/arm_value', FloatList, queue_size=100)
base_pub = rospy.Publisher('movement/base_value', FloatList, queue_size=100)

# Create a rate
rate = rospy.Rate(0.2) # 5 messages per second

# Printing space limit [mm]
lim = [(-50, 50), (-125, 125), (0, 394)] 

# To be printed pose
filename = 'value.txt'
directory = '/home/user'
fullpath = os.path.join(directory, filename)

values = []

with open(fullpath) as f:
	for line in f:
		line = line.split()
		if line:
			if len(line) == 3:
				line = [float(i) for i in line]
				values.append(line)

# Run code in a loop until node is shutdown
while not rospy.is_shutdown():
	for i in range(len(values)):
		movebase_values = []
		for j in range(3):
				
				rospy.sleep(1)
			else:
				arm_pub.publish(values[i])



lim = [(-50, 50), (-125, 125), (0, 394)] 
values = [(0,0,0),(80,0,0), (90,0,0), (100,0,0)]
offset = ()
movebasecounter = 0
movebasetoggle = 0
i = 0
while i < len(values):
    movebase_values = []
    if movebasetoggle == 1:
      i -= 1
    if movebasecounter > 0: 
      values[i] = tuple(map(lambda x, y: x - y, values[i], offset))
      print(values[i])
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
    if (movebase_values == [0,0,0]):
      print(values[i])
      print("movearms") # publish
      movebasetoggle = 0
      print("...")
    else:
      movebasecounter += 1
      print("movebase") # publish
      print(movebase_values)
      offset += tuple(movebase_values)
      print("....")
    i += 1
