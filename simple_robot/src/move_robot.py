#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import Pose

def move_robot():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"  # Change to your planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)
    move_group.set_planning_time(20.0)  # Example: 10 seconds

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == '__main__':
    move_robot()

