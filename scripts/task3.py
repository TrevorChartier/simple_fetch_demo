#!/usr/bin/env python
import sys
import rospy
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from ar_track_alvar_msgs.msg import AlvarMarkers


marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10) # To visualize the detected marker position in RVIZ



def visualize_marker(ar_marker):
    marker = Marker()
    marker.header.frame_id = ar_marker.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "ar_markers"
    marker.id = ar_marker.id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = ar_marker.pose.pose
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
    pose.position.x -= 0.11 # Avoid contact with marker
    arm_group = MoveGroupCommander("arm_with_torso")
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    rospy.loginfo("Move Successful")


def move_to_marker(msg):
    global marker_pub
    try:
        for marker in msg.markers:
            visualize_marker(marker)
            if marker.id == 0:
                rospy.loginfo("AR Tag Detected, Attempting Move")
                move_arm(marker.pose.pose)
                marker_subscriber.unregister()
                rospy.loginfo("Unsubscribed From Marker Detection")
                break
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS Interrupt Exception: %s" % e)
    except Exception as e:
        rospy.logerr("General Exception: %s" % e)


if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node('ar_tag_pose', anonymous=True)
    marker_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, move_to_marker)
    rospy.loginfo("Setup Complete, ready for task 3 marker detection.")
    rospy.sleep(5)
    rospy.spin()
    roscpp_shutdown()
