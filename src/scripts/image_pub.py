#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_publisher():
    rospy.init_node('image_publisher', anonymous=True)
    rate = rospy.Rate(1)  # Adjust the publishing rate as needed

    # Set up the image publisher
    image_pub = rospy.Publisher('image_topic', Image, queue_size=10)
    bridge = CvBridge()

    # Open a camera or load an image (replace this with your image source)


    img = cv2.imread('src/image_process/src/scripts/object.jpg')

    while not rospy.is_shutdown():
        
        rows, cols = img.shape[:2]

# Define the rotation angle
        angle = 0

        # Calculate the rotation matrix
        rotation_matrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)

        # Apply the rotation to the image
        rotated_image = cv2.warpAffine(img, rotation_matrix, (cols, rows))
        cv2.imshow('image', rotated_image)
        cv2.waitKey(10)
        # Convert the image to ROS format
        ros_image = bridge.cv2_to_imgmsg(rotated_image, "bgr8")

        # Publish the image
        image_pub.publish(ros_image)

        rospy.loginfo(f"Image published to {image_pub}")
        rate.sleep()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
