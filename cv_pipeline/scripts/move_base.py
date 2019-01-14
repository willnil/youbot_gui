#!/usr/bin/env python
import rospy
import sys
import message_filters
import roslib; roslib.load_manifest('cv_pipeline')
from geometry_msgs.msg import Twist
from cv_pipeline.msg import FloatList
from collections import deque

class MoveBase:
  
	def __init__(self):
		rospy.init_node('move_base', anonymous=True)
		
		rospy.on_shutdown(self.cleanup)
		#subsribers
		self.distance_ist_sub = rospy.Subscriber('/base_movement/distance', FloatList, self.distanceist_callback) #distance_IST
		self.distance_soll_sub = rospy.Subscriber('/movement/base_value', FloatList, self.distancesoll_callback, queue_size=1) #distance_SOLL
		#publisher
		self.base_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.counter = 0
		self.di_list = deque(maxlen=64) # pointer
		self.rate = rospy.Rate(1)

	def distancesoll_callback(self, data):
		rospy.loginfo("SOLL: {}".format(data.elements))
		ds = data.elements
		self.movebase()

	def distanceist_callback(self, data):
		#rospy.wait_for_message('/movement/base_value', FloatList)
		rospy.loginfo("IST: {}".format(data.elements))
		di = data.elements
		if data.elements != (0, 0):
			self.di_list.appendleft(di)
		#self.movebase()
		self.counter += 1

	def movebase(self, ds, di):
		rospy.loginfo("soll: {}, ist: {}".format(ds, di))
		twist = Twist()
		# linear speed
		twist.linear.x = -0.1
		twist.linear.y = 0
		twist.linear.z = 0
		# angular speed
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = 0
		for i in range(10):
			self.base_pub.publish(twist)
			rospy.sleep(0.1)
		
		twist = Twist()
		self.base_pub.publish(twist)
	
	def cleanup(self):
		print "Shutting down move_base node."

def main(args):
	try:
		MoveBase()
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down move_base node."
		
if __name__=="__main__":
	main(sys.argv)
