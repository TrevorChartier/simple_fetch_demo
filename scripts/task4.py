#!/usr/bin/env python

import sys
import rospy
import copy
import moveit_commander
from moveit_commander import PlanningSceneInterface
import tf
from geometry_msgs.msg import PoseStamped, Pose
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from ar_track_alvar_msgs.msg import AlvarMarkers


class BlockDemo:
    """
    Demonstration class for a pick-and-place task in Gazebo using MoveIt.
    Spawns/deletes objects in Gazebo, plans arm trajectories, and controls
    the gripper through actionlib.
    """

    def __init__(self):
        """
        Initialize ROS node, MoveIt interface, Gazebo services, and arm settings.
        Adds static objects to the planning scene for collision checking.
        """
        rospy.init_node('block_demo')

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_joint = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.scene = PlanningSceneInterface()
        rospy.sleep(1)
        self.add_objects_to_moveit()

        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander("arm_with_torso")
        self.arm.set_pose_reference_frame("base_link")
        self.arm.set_planning_time(5.0)
        self.arm.set_max_velocity_scaling_factor(0.5)

        self.tf_listener = tf.TransformListener()

        self.marker_name = rospy.get_param('~marker_name', 'object_1')
        self.approach_dist = rospy.get_param('approach_distance', 0.35)
        self.moving = False

        self.head_client = actionlib.SimpleActionClient('/head_controller/point_head', PointHeadAction)
        self.head_client.wait_for_server()

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10) # To visualize the detected marker position in RVIZ

        self.marker_subscriber = None

        rospy.loginfo("DemoGazebo node ready")


    def start_detection(self):
        rospy.loginfo("Beginning Task")
        self.marker_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, node.pick_and_place)


    def pick_and_place(self, msg):
        try:
            for marker in msg.markers:
                self.visualize_marker(marker)
                if marker.id == 0:
                    rospy.loginfo("AR Tag Detected, Attempting Move")

                    object_pose = self.__get_object_pose(marker.pose.pose)
                    self.move_to_object(object_pose)
                    self.grasp_object()
                    self.move_object()
                    self.marker_subscriber.unregister()
                    rospy.loginfo("Unsubscribed From Marker Detection")
                    break
        except rospy.ROSInterruptException as e:
            rospy.logerr("ROS Interrupt Exception: %s" % e)
        except Exception as e:
            rospy.logerr("General Exception: %s" % e)
   

    def look_down(self):
        rospy.loginfo("Looking Down")
        goal = PointHeadGoal()
        goal.target.header.frame_id = "base_link"
        goal.target.point.x = 0.6
        goal.target.point.y = 0.0
        goal.target.point.z = 0.5
        goal.min_duration = rospy.Duration(1.0)
        goal.max_velocity = 1.0
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()

    def move_to_object(self, object_pose):
        """
        Move the arm to a pre-grasp pose in front of the target object.
        Uses Gazebo service to query the object pose, then computes an
        offset along the object's x-axis for approach.
        """
        self.open_gripper()

        q = object_pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        rot_matrix = tf.transformations.quaternion_matrix(quat)[:3, :3]
        marker_x = rot_matrix[:, 0]
        offset = -marker_x * self.approach_dist

        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = object_pose.position.x + float(offset[0])
        target_pose.pose.position.y = object_pose.position.y + (-0.01)
        target_pose.pose.position.z = object_pose.position.z + float(offset[2])
        target_pose.pose.orientation = object_pose.orientation

        rospy.loginfo("Moving to object")
        self.move_to_pose(target_pose)


    def __get_object_pose(self, marker_pose):
        """
        Determine pose of associated object from ar marker pose
        """

        pose = Pose()
        pose.position.x = marker_pose.position.x
        pose.position.y = marker_pose.position.y
        pose.position.z = marker_pose.position.z - 0.05  # shift from top face to center

        # Orientation: gripper approaching from front (-X)
        import tf
        q = tf.transformations.quaternion_from_euler(0, 0, 0)  
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

        return pose


    def visualize_marker(self, ar_marker):
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
        self.marker_pub.publish(marker)

    def grasp_object(self):
        """
        Execute a grasp motion: advance closer, close the gripper,
        and attach the object in MoveIt. Spawns a fixed joint in Gazebo
        to simulate grasping.
        """
        rospy.loginfo("Moving to Grasp Pose")

        grasp_pose = copy.deepcopy(self.arm.get_current_pose())
        grasp_pose.pose.position.x += .25
        self.move_direct_to_pose(grasp_pose)

        # Make sure object exists in planning scene before attaching
        if "object_1" not in self.scene.get_known_object_names():
            rospy.logwarn("object_1 not in planning scene, adding before attach")
            object_pose = PoseStamped()
            object_pose.header.frame_id = "base_link"
            object_pose.pose.position.x = 0.65
            object_pose.pose.position.y = -0.3
            object_pose.pose.position.z = 0.55
            self.scene.add_box("object_1", object_pose, size=(0.08, 0.05, 0.11))
            rospy.sleep(0.5)

        self.scene.attach_box(
            link="gripper_link",
            name="object_1",
            touch_links=["gripper_link", "l_gripper_finger_link", "r_gripper_finger_link"]
        )

        self.close_gripper()
        rospy.sleep(0.2)


    def move_object(self):
        """
        Lift the object, move it to a new location, lower it,
        then release the grasp and remove the fixed joint from Gazebo.
        """
        goal_pose = copy.deepcopy(self.arm.get_current_pose())
        goal_pose.pose.position.z += 0.2
        self.move_to_pose(goal_pose)

        goal_pose.pose.position.y += 0.6
        goal_pose.pose.position.x += 0.2
        self.move_to_pose(goal_pose)

        goal_pose.pose.position.z -= 0.1
        self.move_direct_to_pose(goal_pose)

        self.open_gripper()


    def close_gripper(self):
        """Command the gripper to fully close."""
        self.command_gripper(position=0)


    def open_gripper(self):
        """Command the gripper to fully open."""
        self.command_gripper(position=1)


    def command_gripper(self, position):
        """
        Send a position command to the gripper action server.
        Args:
            position (float): Desired gripper opening (0 = closed, 1 = open).
        """
        rospy.loginfo("Waiting to command gripper")
        client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
        client.wait_for_server()

        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = 300.0
        client.send_goal(goal)
        client.wait_for_result()


    def move_to_pose(self, target_pose):
        """
        Plan and execute a MoveIt trajectory to reach a given pose.
        Args:
            target_pose (PoseStamped): Desired target pose.
        """
        rospy.loginfo("Planning move...")
        try:
            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(target_pose.pose)
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            if success:
                rospy.loginfo("Move executed succesfully")
            else:
                rospy.logwarn("MoveIt planner failed")
        except Exception as e:
            rospy.logerr("Exception during MoveIt call")
        finally:
            rospy.sleep(1.0)
            self.moving = False

    def move_direct_to_pose(self, target_pose):
        """
        Execute a Cartesian path directly to a target pose (no planning).
        Args:
            target_pose (PoseStamped): Desired target pose.
        """
        rospy.loginfo("Moving Directly to Pose")
        waypoints = [copy.deepcopy(target_pose.pose)]
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    def add_objects_to_moveit(self):
        """
        Add static objects (tables and initial block) to the MoveIt planning scene
        so that the planner is aware of collisions.
        """
        table1_pose = PoseStamped()
        table1_pose.header.frame_id = "base_link"
        table1_pose.pose.position.x = 0.8
        table1_pose.pose.position.y = 0.3
        table1_pose.pose.position.z = 0.25
        self.scene.add_box("table_1", table1_pose, size=(0.5, 0.5, 0.5))

        table2_pose = PoseStamped()
        table2_pose.header.frame_id = "base_link"
        table2_pose.pose.position.x = 0.8
        table2_pose.pose.position.y = -0.3
        table2_pose.pose.position.z = 0.25
        self.scene.add_box("table_2", table2_pose, size=(0.5, 0.5, 0.5))

        object_pose = PoseStamped()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.position.x = 0.65
        object_pose.pose.position.y = -0.3
        object_pose.pose.position.z = 0.55
        self.scene.add_box("object_1", object_pose, size=(0.08, 0.05, 0.11))


if __name__ == '__main__':
    node = BlockDemo()
    node.look_down()
    node.start_detection()
    rospy.spin()
