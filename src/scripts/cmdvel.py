#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import copy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose

linear_velocity_x=0.0
linear_velocity_y=0.0
linear_velocity_z=0.0

def move_to_pose(pose_goal, group):
    group.set_pose_target(pose_goal)
    plan = group.go(wait=False)
    return plan

def cmd_vel_callback(msg):
    # Callback function to process incoming /cmd_vel messages
    linear_velocity_x = msg.linear.x
    linear_velocity_y =msg.linear.y
    linear_velocity_z=msg.linear.z
    angular_velocity = msg.angular.z
    print(f"Linear Velocity: {linear_velocity_x}, Angular Velocity: {angular_velocity}")
    group_name = "manipulator"  # replace with your robot's planning group name
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_max_velocity_scaling_factor(0.20)
    group.set_max_acceleration_scaling_factor(0.20)
    
    pose_goal = Pose()
    pose_goal.position.x = float(linear_velocity_x)# set x-coordinate
    pose_goal.position.y = float(linear_velocity_y)# set y-coordinate
    pose_goal.position.z = float(linear_velocity_z)
    pose_goal.orientation.x = 1.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0  # Axis of rotation (Z-axis)
    pose_goal.orientation.w = 0.0 

    # Move the robot to the goal pose
    plan = move_to_pose(pose_goal, group)
    

def cmd_vel_subscriber():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_subscriber', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()


    # Create a subscriber to the /cmd_vel topic with the specified message type
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    # Keep the script running to receive messages
    rospy.spin()

if __name__ == '__main__':
    try:
        # Run the subscriber node
        cmd_vel_subscriber()
    except rospy.ROSInterruptException:
        pass
