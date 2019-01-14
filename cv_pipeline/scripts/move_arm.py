#!/usr/bin/env python

## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
from math import cos, sin, pi
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf import transformations
from std_msgs.msg import String
from cv_pipeline.msg import FloatList

class MoveArm:

	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)

		rospy.on_shutdown(self.cleanup)
		# Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
		self.robot = moveit_commander.RobotCommander()
		# Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
		self.scene = moveit_commander.PlanningSceneInterface()
		# Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints. 
		# This interface can be used to plan and execute motions on the youbot arms
		self.group = moveit_commander.MoveGroupCommander("arm_1")
		# We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
		# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

	def start(self):
		# Start the arm in the work pose stored in the SRDF file
  		self.group.set_named_target("ready_3")
		self.group.go()
		rospy.sleep(2)

	def listener(self):
		rospy.Subscriber('movement/arm_value', FloatList, self.pose_callback, queue_size=1)

	def pose_callback(self, data):
		# distance transformations from 3D pen printing head to youbot eef link
		df = [-0.0445, 0, 0.147]
		# default origin of printing space 
		origin = [0.5545, 0, -0.120] # odom frame
		# pose coordinates
		coor = data.elements

		# Planning to a Pose goal
  		# We can plan a motion for this group to a desired pose for the end-effector
  		print "============ Generating plans"
		
		# pose
		pose=geometry_msgs.msg.Pose()
		pose.position.x = origin[0] + df[0] + coor[1]*0.001
		pose.position.y = origin[1] + df[1] + coor[0]*0.001
		pose.position.z = origin[2] + df[2] + coor[2]*0.001
	  
		q = transformations.quaternion_from_euler(0, pi/2, 0, "sxyz")

		# orientation
		pose.orientation.x = q[0]
		pose.orientation.y = q[1]
		pose.orientation.z = q[2]
		pose.orientation.w = q[3]

		eef_link = self.group.get_end_effector_link()

		# set pose
		self.group.set_pose_target(pose, eef_link)
		# set pose goal tolerance
		self.group.set_goal_tolerance(0.01)

		# Now, we call the planner to compute the plan, asking move_group to actually move the robot
		self.group.go(wait=True)

	def cleanup(self):
		self.group.stop()
		self.group.clear_pose_targets()
		moveit_commander.roscpp_shutdown()
		print "============ STOPPING"

if __name__=='__main__':
	rospy.init_node('move_group_python_interface', anonymous=True)
	try:
		move_arm = MoveArm()
		move_arm.start()
		move_arm.listener()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

