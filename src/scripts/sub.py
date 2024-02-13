#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import tf
from geometry_msgs.msg import Pose

def callback(image_msg):
    """This function is called to handle the subscribed messages

    Args:
        image_msg (Image): message type Image from sensor_msgs
    """
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        cv2.imshow('ROS Image Subscriber', cv_image)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cv2.imshow("hsv image",hsv_image)
        lower_bound_color=np.array([0, 0, 200])  #([0, 0, 200]) {'hmin': 88, 'smin': 0, 'vmin': 0, 'hmax': 122, 'smax': 255, 'vmax': 255}
        upper_bound_color=np.array([180, 30, 255])  #([180, 30, 255])
    #define a mask using the lower and upper bounds of the yellow color 
        mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
        cv2.imshow('mask',mask)
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(mask, 
                                            cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
        # while(contours[0]==0):
            # pass
        # print(f"the contours are {contours} and type is {type(contours)}")
        black_image = np.zeros([mask.shape[0], mask.shape[1],3],'uint8')
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            box_center, box_size, box_angle = cv2.minAreaRect(largest_contour)
            # cv2.drawContours(cv_image, contours, -1, (150,250,150), 1)
            cv2.drawContours(cv_image, largest_contour, -1, (150,250,150), 1)
            cv2.drawContours(black_image, largest_contour, -1, (150,250,150), 1)
            box_corners = cv2.boxPoints((box_center, box_size, box_angle))
        # Convert the corners to integer values
            box_corners = np.int0(box_corners)
            cv2.drawContours(cv_image, [box_corners], 0, (255, 0, 0), 2)
            cv2.drawContours(black_image, [box_corners], 0, (255, 0, 0), 2)
            list(box_center)
            cv2.circle(cv_image, (int(box_center[0]), int(box_center[1])), 5, (0, 255, 0), -1)
            for corner in box_corners:
                cv2.circle(cv_image, tuple(corner), 5, (0, 0, 255), -1)
            for i, corner in enumerate(box_corners):
                cv2.putText(cv_image, str(i), tuple(corner), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
            midpoint = ((box_corners[0][0] + box_corners[1][0]) // 2, (box_corners[0][1] + box_corners[1][1]) // 2)
            # Draw a line from the center to the midpoint
            cv2.line(cv_image, tuple(map(int, box_center)), tuple(map(int, midpoint)), (0, 255, 0), 2)
            cv2.line(cv_image, tuple(map(int, (box_center[0] - 50, box_center[1]))), tuple(map(int, (box_center[0] + 50, box_center[1]))), (255, 0, 0), 2)

            # Calculate the angle between the two lines
            dx = midpoint[0] - box_center[0]
            dy = midpoint[1] - box_center[1]
            angle_rad = 2*np.pi - np.arctan2(dy, dx)
            angle_deg = np.degrees(angle_rad)
            if angle_deg > 180:
                angle_deg -= 180
            box_angle= abs(box_angle)
            # Display the image with red points, indexes, lines, and angle information
            cv2.putText(cv_image, f'Angle: {box_angle:.2f} degrees', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2, cv2.LINE_AA)

            print ("number of contours: {}".format(len(contours)))
            print(f"box_center={box_center}, box_size={box_size}, box_angle={box_angle}, box_corners={box_corners}, angle_deg={angle_deg}")
            cv2.imshow("RGB Image Contours",cv_image)
            # cv2.imshow("Black Image Contours",black_image)
            box_x=box_center[0]
            box_y=box_center[1]

            box_z=cv_image[int(box_center[1]),int(box_center[0])]
            print(f"coordinates inside are {box_x} and {box_y} and {box_z}")
            angle_rad = np.radians(box_angle)
            object_size_meters = 0.08  # 5 cm converted to meters
            object_size_pixels = 174  # Replace with the actual size in pixels provided by OpenCV           

            # pixel_size_meters = object_size_meters / object_size_pixels         
            pixel_size_meters=0.0004597701149425287
            prin_x=640
            prin_y=360
            # Assuming you have the pixel coordinates of the center pointpoko  
            center_pixel_x = prin_x- box_center[0]  # Replace with the actual x-coordinate
            center_pixel_y = box_center[1] - prin_y # Replace with the actual y-coordinate 
            
            # if box_center[0] < prin_x:
            #     center_pixel_x = box_center[0] - prin_x # Replace with the actual x-coordinate
            #     center_pixel_y = box_center[1] - prin_y
                
                 
            # if center_pixel_x<0 and 
            
            # Convert pixel coordinates to meters
            center_point_x_meters = center_pixel_x * pixel_size_meters
            center_point_y_meters = center_pixel_y * pixel_size_meters        
            
            
            print(f"pixel_size_meters={pixel_size_meters}, Center point in meters: ({center_point_x_meters}, {center_point_y_meters})")

            pose_msg = Pose()
            pose_msg.position.x = center_point_x_meters # Set your actual x-coordinate
            pose_msg.position.y = center_point_y_meters # Set your actual y-coordinate
            pose_msg.position.z = 0.5  # Set your actual z-coordinate

            pose_msg.orientation.x = 0.0  # Set your actual orientation x
            pose_msg.orientation.y = 0.0  # Set your actual orientation y
            pose_msg.orientation.z = 0.0  # Set your actual orientation z
            pose_msg.orientation.w = 1.0  # Set your actual orientation w
            
            pose_pub.publish(pose_msg)
        else:   
            print("no contours")
        cv2.waitKey(10)

    except CvBridgeError as error:
        print(error)

if __name__=="__main__":
    rospy.init_node("image_subscriber", anonymous=True)
    print("Subscribe images from topic /image_raw ...")

    image_subcriber = rospy.Subscriber("/camera/color/image_raw", Image, callback) #/camera/color/image_raw     image_topic

    # Create a publisher for the PoseStamped messages
    pose_pub = rospy.Publisher('/camera_coord', Pose, queue_size=10)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
