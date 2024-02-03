#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose

def move_to_pose(pose_goal, group):
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    return plan

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # replace with your robot's planning group name
    group = moveit_commander.MoveGroupCommander(group_name)

    # Define the goal pose
    pose_goal = Pose()
    pose_goal.position.x = 0.0 # set x-coordinate
    pose_goal.position.y = 0.307 # set y-coordinate
    pose_goal.position.z = 0.527
    pose_goal.orientation.x = 1.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0  # Axis of rotation (Z-axis)
    pose_goal.orientation.w = 0.0 

    # Move the robot to the goal pose
    plan = move_to_pose(pose_goal, group)

    # Print the execution result
    if plan:
        print("Move successful!")
    else:
        print("Move failed!")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
    
# sudo ip link set can0 up type can bitrate 1000000
# roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=false pipeline:=ompl safety_hw:=pss4000
# roslaunch realsense2_camera rs_camera.launch

#length and width 9,4.5 cm 
