#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import tf
from geometry_msgs.msg import PoseStamped

rospy.init_node('task2_transform_frame', anonymous=False)
moveit_commander.roscpp_initialize(sys.argv)

arm = moveit_commander.MoveGroupCommander("arm_with_torso")

base_pose = PoseStamped()
base_pose.header.frame_id = "base_link"
base_pose.pose.position.x = 0.5
base_pose.pose.position.y = 0.0
base_pose.pose.position.z = 0.9
base_pose.pose.orientation.x = 0.0
base_pose.pose.orientation.y = 0.707
base_pose.pose.orientation.z = 0.0
base_pose.pose.orientation.w = 0.707

listener = tf.TransformListener()
listener.waitForTransform("head_camera_link", "base_link", rospy.Time(), rospy.Duration(4.0))

camera_pose = listener.transformPose("head_camera_link", base_pose)

arm.set_pose_target(camera_pose)

arm.go(wait=True)

arm.clear_pose_targets()

moveit_commander.roscpp_shutdown()