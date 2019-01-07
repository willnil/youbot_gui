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
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##

  # We can get the name of the reference frame for this robot:
  planning_frame = group.get_planning_frame()
  print "============ Reference frame: %s" % planning_frame

  # We can also print the name of the end-effector link for this group:
  eef_link = group.get_end_effector_link()
  print "============ End effector: %s" % eef_link

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  # Start the arm in the work pose stored in the SRDF file
  group.set_named_target("folded")
  group.go()
  rospy.sleep(2)

  print "============ Printing robot pose"
  print group.get_current_pose()
  print "============"
  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"

  ## pose
  pose=geometry_msgs.msg.Pose()
  pose.position.x = 0.510
  pose.position.y = 0.0006
  pose.position.z = 0.046
	  
  #q = transformations.quaternion_from_euler(pi/2, pi/2, 0, "sxyz")

  ## orientation
  pose.orientation.x=0.0005
  pose.orientation.y=0.7136
  pose.orientation.z=0.0187
  pose.orientation.w=0.7001

  ## set pose
  group.set_pose_target(pose, eef_link)
  ## set pose goal tolerance 
  group.set_goal_tolerance(0.01)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  group.go(wait=True)

  group.stop()

  group.clear_pose_targets()

  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)

 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again).
  ##print "============ Visualizing plan1"
  ##display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  ##display_trajectory.trajectory_start = robot.get_current_state()
  ##display_trajectory.trajectory.append(plan1)
  ##display_trajectory_publisher.publish(display_trajectory);

  ##print "============ Waiting while plan1 is visualized (again)..."
  ##rospy.sleep(5)

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface()
  except rospy.ROSInterruptException:
    pass

