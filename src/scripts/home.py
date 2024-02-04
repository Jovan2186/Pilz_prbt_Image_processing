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
    group.set_max_velocity_scaling_factor(0.20)
    group.set_max_acceleration_scaling_factor(0.20)

    # Define the pick pose
    pick_pose = Pose()
    pick_pose.position.x = 0.0  # set x-coordinate for pick position
    pick_pose.position.y = 0.307  # set y-coordinate for pick position
    pick_pose.position.z = 0.527
    pick_pose.orientation.x = 1.0
    pick_pose.orientation.y = 0.0
    pick_pose.orientation.z = 0.0  # Axis of rotation (Z-axis)
    pick_pose.orientation.w = 0.0 

    # Move the robot to the pick pose
    pick_plan = move_to_pose(pick_pose, group)

    if not pick_plan:
        print("Pick failed!")
        moveit_commander.roscpp_shutdown()
        return

    print("Pick successful!")

    # Define the place pose
    place_pose = Pose()
    place_pose.position.x = 0.1  # set x-coordinate for place position
    place_pose.position.y = 0.307  # set y-coordinate for place position
    place_pose.position.z = 0.527
    place_pose.orientation.x = 1.0
    place_pose.orientation.y = 0.0
    place_pose.orientation.z = 0.0  # Axis of rotation (Z-axis)
    place_pose.orientation.w = 0.0 

    # Move the robot to the place pose
    place_plan = move_to_pose(place_pose, group)

    # Print the execution result for place
    if place_plan:
        print("Place successful!")
    else:
        print("Place failed!")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

# sudo ip link set can0 up type can bitrate 1000000
# roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=false pipeline:=ompl safety_hw:=pss4000
# roslaunch realsense2_camera rs_camera.launch

#length and width 9,4.5 cm 
