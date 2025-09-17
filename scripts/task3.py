#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from ar_track_alvar_msgs.msg import AlvarMarkers

def move_to_marker():
    roscpp_initialize(sys.argv)
    rospy.init_node("fetch_move_to_marker", anonymous=True)

    # MoveIt setup
    arm = MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    # TF setup
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("Waiting for AR markers...")

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            # Get detected markers
            markers_msg = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers, timeout=1.0)

            if not markers_msg.markers:
                rospy.logwarn("No AR markers detected yet...")
                rate.sleep()
                continue

            # Pick the first marker (or choose by ID)
            marker_pose_cam = markers_msg.markers[0].pose

            # Transform marker pose to base_link frame
            marker_pose_base = tf_buffer.transform(marker_pose_cam, "base_link", rospy.Duration(1.0))

            # Optional: offset along z-axis to hover above marker
            marker_pose_base.pose.position.z += 0.05

            rospy.loginfo("Planning to AR marker...")
            arm.set_pose_target(marker_pose_base)
            plan = arm.plan()

            if plan and len(plan.joint_trajectory.points) > 0:
                rospy.loginfo("Executing plan...")
                arm.execute(plan, wait=True)
            else:
                rospy.logwarn("No valid plan found. Retrying...")

            rate.sleep()

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed, retrying...")
            rate.sleep()
        except rospy.ROSException:
            rospy.logwarn("Waiting for AR marker message...")
            rate.sleep()

    roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_to_marker()
    except rospy.ROSInterruptException:
        pass
