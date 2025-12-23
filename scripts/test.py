#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

def callback(msg):
    for detection in msg.detections:
        print("ID:", detection.id)
        print("Pose:", detection.pose.pose.pose.position)

rospy.init_node('tag_listener')
sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)
rospy.spin()
