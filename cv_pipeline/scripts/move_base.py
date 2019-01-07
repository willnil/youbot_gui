#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

if __name__=="__main__":
	sub = rospy.Subscriber('', Float32)
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('move_base')

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
