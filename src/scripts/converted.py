#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose,PoseStamped
import tf

def camera_coord_callback(camera_pose):
    # This function will be called whenever a new Pose message is received
    # Wait for the transformation to be available
    target_frame = "prbt_base"  # Replace with your robot's base frame
    source_frame = "prbt_tool0"  # Replace with your camera frame
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    rospy.loginfo("Received Camera Pose:\n%s" % camera_pose)

    # camera_pose_stamped = tf.TransformerROS().fromTranslationRotation(
    #     (camera_pose.position.x, camera_pose.position.y, camera_pose.position.z),
    #     (camera_pose.orientation.x, camera_pose.orientation.y,
    #      camera_pose.orientation.z, camera_pose.orientation.w))
    camera_pose_stamped = PoseStamped()
    camera_pose_stamped.header.frame_id = "prbt_tool0"
    camera_pose_stamped.pose = camera_pose
    
    robot_base_pose_stamped = listener.transformPose("prbt_base", camera_pose_stamped)

        # Extract the transformed pose
    robot_base_pose = robot_base_pose_stamped.pose
    print(f"robot_pose_stamped: {robot_base_pose}")

if __name__ == '__main__':
    rospy.init_node('pose_subscriber', anonymous=True)

    # Subscribe to camera coordinates
    rospy.Subscriber('/camera_coord', Pose, camera_coord_callback)
    listener = tf.TransformListener()



    # Spin ROS node
    rospy.spin()
