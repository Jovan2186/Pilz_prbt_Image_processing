#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose,PoseStamped
import tf
import numpy as np
def camera_coord_callback(camera_pose):
    
    matrix_4x4 = np.array([[1.0, 0.0000000, 0.00000, 0.0],
                   [0.00000, -1.00000, 0.0000000, 0.352],
                   [0.0000000, 0.000000, -1.00000,0.487],
                   [0, 0, 0, 1]])
    matrix_4x1 = np.array([[camera_pose.position.x],
                   [camera_pose.position.y],
                   [0.0],
                   [1]])
    result = np.dot(matrix_4x4, matrix_4x1)
    print(f"result: {result}")


if __name__ == '__main__':
    rospy.init_node('pose_subscriber', anonymous=True)

    # Subscribe to camera coordinates
    rospy.Subscriber('/camera_coord', Pose, camera_coord_callback)
    listener = tf.TransformListener()



    # Spin ROS node
    rospy.spin()

        # matrix_4x4 = np.array([[1.0, 0.0000000, 0.00000, 0.0],
        #                [0.00000, -1.00000, 0.0000000, 0.352],
        #                [0.0000000, 0.000000, -1.00000,0.487],
        #                [0, 0, 0, 1]])
        # matrix_4x1 = np.array([[0.65],
        #                [box_x],
        #                [box_y],
        #                [1]])
        # result = np.dot(matrix_4x4, matrix_4x1)