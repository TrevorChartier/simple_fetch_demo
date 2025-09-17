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


class Task3Gazebo:
    def __init__(self):
        rospy.init_node('task3_gazebo_marker')

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

        rospy.loginfo("DemoGazebo node ready")


    def move_to_object(self):
        self.open_gripper()
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Get marker pose
        resp = get_model_state(self.marker_name, 'world')
        marker_pose = resp.pose

        # Compute approach offset
        q = marker_pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        rot_matrix = tf.transformations.quaternion_matrix(quat)[:3, :3]
        marker_x = rot_matrix[:, 0]
        offset = -marker_x * self.approach_dist

        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = marker_pose.position.x + float(offset[0])
        target_pose.pose.position.y = marker_pose.position.y + (-0.01)
        target_pose.pose.position.z = marker_pose.position.z + float(offset[2])
        target_pose.pose.orientation = marker_pose.orientation

        rospy.loginfo("Moving to object")
        self.move_to_pose(target_pose)

    def grasp_object(self):
        rospy.loginfo("Moving to Grasp Pose")

        grasp_pose = copy.deepcopy(self.arm.get_current_pose())
        grasp_pose.pose.position.x += .15
        self.move_direct_to_pose(grasp_pose)

        self.scene.attach_box(
            link="gripper_link",
            name="object_1",
            touch_links=["gripper_link", "l_gripper_finger_link", "r_gripper_finger_link"]
        )

        self.close_gripper()

        fixed_joint_sdf = open("models/object/fixed_joint.sdf", "r").read()
        joint_pose = Pose()
        self.spawn_joint("grasp_joint", fixed_joint_sdf, "", joint_pose, "world")
        rospy.sleep(0.1)


    def move_object(self):
        goal_pose = copy.deepcopy(self.arm.get_current_pose())
        goal_pose.pose.position.z += 0.2
        self.move_to_pose(goal_pose)

        goal_pose.pose.position.y += 0.6
        goal_pose.pose.position.x += 0.2
        self.move_to_pose(goal_pose)

        goal_pose.pose.position.z -= 0.15
        self.move_direct_to_pose(goal_pose)

        self.delete_model("grasp_joint")

        self.open_gripper()



    def close_gripper(self):
        self.command_gripper(position=0)
    
    def open_gripper(self):
        self.command_gripper(position=1)

    def command_gripper(self, position):
        rospy.loginfo("Waiting to command gripper")
        client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
        client.wait_for_server()

        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = 300.0
        client.send_goal(goal)
        client.wait_for_result()

    def move_to_pose(self, target_pose):
        # Move arm
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
        rospy.loginfo("Moving Directly to Pose")
        waypoints = []
        waypoints.append(copy.deepcopy(target_pose.pose))
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()


    def add_objects_to_moveit(self):

        # Add tables
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
    node = Task3Gazebo()
    node.move_to_object()
    node.grasp_object()
    node.move_object()
    rospy.spin()