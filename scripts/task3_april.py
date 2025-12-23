#!/usr/bin/env python

""" This is identical to task3.py except for it uses apriltag """
import sys
import rospy
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from apriltag_ros.msg import AprilTagDetectionArray  # Updated message type

marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

def visualize_marker(tag_detection):
    # Use the first ID in the detection (most tags only have one)
    tag_id = tag_detection.id[0] if tag_detection.id else 0

    marker = Marker()
    marker.header.frame_id = tag_detection.pose.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "apriltag_markers"
    marker.id = tag_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = tag_detection.pose.pose.pose
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()
    marker_pub.publish(marker)

def move_arm(pose):
    pose.position.x -= 0.11  # Avoid contact with marker
    arm_group = MoveGroupCommander("arm_with_torso")
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    rospy.loginfo("Move Successful")

def move_to_marker(msg):
    global marker_subscriber
    try:
        for detection in msg.detections:
            visualize_marker(detection)
            # Check tag ID 0 
            if 0 in detection.id:
                rospy.loginfo("AprilTag Detected, Attempting Move")
                move_arm(detection.pose.pose.pose)
                marker_subscriber.unregister()
                rospy.loginfo("Unsubscribed From Tag Detection")
                break
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS Interrupt Exception: %s" % e)
    except Exception as e:
        rospy.logerr("General Exception: %s" % e)

if __name__ == "__main__":
    rospy.loginfo("Starting Custom Node")
    roscpp_initialize(sys.argv)
    rospy.init_node('apriltag_pose', anonymous=True)
    marker_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, move_to_marker)
    rospy.loginfo("Setup Complete, ready for task 3 tag detection.")
    rospy.sleep(5)
    rospy.spin()
    roscpp_shutdown()
