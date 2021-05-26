#!/usr/bin/env python
from __future__ import print_function
import sys
import os
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

import os

if __name__ == '__main__':
    rospy.init_node('kitti_publisher')
    cv_bridge = CvBridge()
    left_pub = rospy.Publisher("/camera/left/image_raw", Image, queue_size=2000)
    right_pub = rospy.Publisher("/camera/right/image_raw", Image, queue_size=2000)
    
    path_name = "/media/zlq/Data/KITTI_odom_jpg/sequences/" + sys.argv[1]
    image_index = 0
    rate = rospy.Rate(10)
    cv2.namedWindow("left")
    cv2.moveWindow("left", 100, 100)
    # cv2.waitKey(0)
    time.sleep(2)
    while True:
        print(image_index)
        left_img_path = path_name + "/image_2/%06d.jpg"%image_index
        right_img_path = path_name + "/image_3/%06d.jpg"%image_index
        
        if (not os.path.isfile(left_img_path)) or (not os.path.isfile(right_img_path)):
            break

        left_img = cv2.imread(left_img_path)
        right_img = cv2.imread(right_img_path)
        # left_img = cv2.flip(left_img,1,dst=None)
        # right_img = cv2.flip(right_img,1,dst=None)

        pub_ros_time = rospy.get_rostime()
        left_msg = cv_bridge.cv2_to_imgmsg(left_img, '8UC3')
        left_msg.header.stamp = pub_ros_time
        left_pub.publish(left_msg)
	
        right_msg = cv_bridge.cv2_to_imgmsg(right_img, '8UC3')
        right_msg.header.stamp = pub_ros_time
        right_pub.publish(right_msg)

        

        rate.sleep()
        image_index+=1

        cv2.imshow("left", left_img)
        input_key = cv2.waitKey(10)

        if input_key == 27:
            break
