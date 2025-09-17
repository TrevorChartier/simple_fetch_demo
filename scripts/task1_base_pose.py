#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

rospy.init_node('task1_base_pose', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

arm = moveit_commander.MoveGroupCommander("arm_with_torso")

target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.2
target_pose.orientation.x = 0.0
target_pose.orientation.y = 0.707
target_pose.orientation.z = 0.0
target_pose.orientation.w = 0.707


arm.set_pose_target(target_pose)

arm.go(wait=True)

arm.clear_pose_targets()

moveit_commander.roscpp_shutdown()