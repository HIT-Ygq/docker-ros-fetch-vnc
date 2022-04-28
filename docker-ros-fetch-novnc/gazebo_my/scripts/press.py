#!/usr/bin/env python

import sys
import rospy
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
# arm_intermediate_positions  = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]
arm_intermediate_positions  = [0, 1.40, -3.14, 1.72, 0.0, 1.66, 0.0]
arm_tuck_positions  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
arm_joint_positions = [0, 1, -3.14, 2, 0.0, -1, 0.0]
arm_press_positions = [0, 0.8, -3.14, 1.7, 0.0, -0.9, 0.0]

if __name__ == "__main__":
    rospy.init_node("press")

    rospy.loginfo("Waiting for arm_controller...")
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.loginfo("Waiting for gripper_controller...")
    gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper_client.wait_for_server()
    rospy.loginfo("...connected.")

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = arm_intermediate_positions
    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(4.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = arm_joint_positions
    trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[1].time_from_start = rospy.Duration(7.5)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = 0.0

    rospy.loginfo("Setting positions...")
    arm_client.send_goal(arm_goal)
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(5.0))
    arm_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")
    rospy.sleep(1)

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = arm_press_positions
    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(4.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = arm_joint_positions
    trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[1].time_from_start = rospy.Duration(7.5)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    rospy.loginfo("Setting positions...")
    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")
    rospy.sleep(1)

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = arm_intermediate_positions
    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(4.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = arm_tuck_positions
    trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)
    trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
    trajectory.points[1].time_from_start = rospy.Duration(7.5)

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = trajectory
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = 0.1

    rospy.loginfo("Setting positions...")
    arm_client.send_goal(arm_goal)
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(5.0))
    arm_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")