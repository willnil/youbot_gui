#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from cv_pipeline.msg import FloatList

class move_base:

  def __init__(self):
	#subsriber
	distance_ist_sub = rospy.Subscriber('/base_movement/distance', FloatList, self.distanceist_callback) #distance_IST
	distance_soll_sub = rospy.Subscriber('', FloatList, self.distancesoll_callback) #distance_SOLL
	#publisher
	pub = rospy.Publisher('cmd_vel', Twist)

if __name__=="__main__":
	rospy.init_node('move_base', anonymous=True)
	while not rospy.is_shutdown():
        	move_base = move_base()
        	rospy.spin()



	x = 0
	y = 0
	th = 0

	try:
		while(1):
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				th = moveBindings[key][2]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
	
			else:
				x = 0
				y = 0
				th = 0

			twist = Twist()
			twist.linear.x = x*speed 
			twist.linear.y = y*speed 
			twist.linear.z = 0

			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print e
