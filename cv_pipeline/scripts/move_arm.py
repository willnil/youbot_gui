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

# distance transformations from 3D pen printing head to youbot eef link
df = [-0.0445, 0, 0.147]

# default origin of printing space 
origin = [0.5545, 0, -0.101] # odom frame

def move_group_python_interface():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  ## Initialize the node for this script
  rospy.init_node('move_group_python_interface',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints. This interface can be used to plan and execute 
  ## motions on the youbot arms
  group = moveit_commander.MoveGroupCommander("arm_1")

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(10)
  print "============ Starting... "

  eef_link = group.get_end_effector_link()

  # Start the arm in the work pose stored in the SRDF file
  group.set_named_target("ready_3")
  group.go()
  rospy.sleep(2)

  print "============ Printing robot pose"
  print group.get_current_pose()
  print "============"
  ## Planning to a Pose goal
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plans"

  ## pose
  pose=geometry_msgs.msg.Pose()
  pose.position.x = origin[0] + df[0]
  pose.position.y = origin[1] + df[1]
  pose.position.z = origin[2] + df[2]
	  
  q = transformations.quaternion_from_euler(0, pi/2, 0, "sxyz")

  ## orientation
  pose.orientation.x = q[0]
  pose.orientation.y = q[1]
  pose.orientation.z = q[2]
  pose.orientation.w = q[3]

  ## set pose
  group.set_pose_target(pose, eef_link)
  ## set pose goal tolerance
  group.set_goal_tolerance(0.01)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## asking move_group 
  ## to actually move the robot
  group.go(wait=True)

  group.stop()

  group.clear_pose_targets()

  print "============ Waiting while RVIZ displays plan..."
  rospy.sleep(5)

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface()
  except rospy.ROSInterruptException:
    pass

