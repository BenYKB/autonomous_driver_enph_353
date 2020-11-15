#!/usr/bin/env python

import cv2
#import csv
import numpy as np
from sensor_msgs.msg import Image
#import os
#import pyqrcode
#import random
#import string
import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# Code used for autonomous control of rover

class controller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self._image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback=self._image_callback, queue_size=1)
        self._twist_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self._image_pub = rospy.Publisher('R1/debug_image', Image, queue_size=1)
        self._bridge = CvBridge()
        

    def _image_callback(self, image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            raise e
            return
        
        # TODO: process image to follow path
        row_max, col_max, channels = cv_image.shape
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        road_mask = cv2.inRange(hsv_frame, (0,0,80), (10,10,90))
        line_mask = cv2.inRange(hsv_frame, (0,0,253), (2,2,255))
        cross_walk_red_mask = cv2.inRange(hsv_frame, (0,250,250), (2,255,255))     

        
        line_col_start = 1*(col_max//3)
        line_row_start = -1*(row_max//3)
        cropped_line_mask = line_mask[line_row_start:, line_col_start:]

        M_road = cv2.moments(road_mask)
        M_line = cv2.moments(cropped_line_mask)

        alpha_line = 0.3

        out_frame = road_mask

        error = 0
        if M_road["m00"] != 0:
            cX = int(M_road["m10"] / M_road["m00"])
            cY = int(M_road["m01"] / M_road["m00"])
            out_frame = cv2.circle(road_mask, (cX, cY), 40, (0,100,0), -1)
            error = cX - col_max / 2

            if M_line["m00"] != 0:
                cX = int(M_line["m10"] / M_line["m00"])
                cY = int(M_line["m01"] / M_line["m00"])

                out_frame = cv2.circle(road_mask, (cX+ (col_max//3), (row_max//3) + cY), 40, (0,100,0), -1)

                error = (1 - alpha_line) * error + alpha_line * (cX - (3 * line_col_start) / 4)



            # if -10 < error < 10:
            #     error =-10

        move_cmd = Twist()
        move_cmd.linear.x = 0.1 - error * 0.0001
        move_cmd.angular.z = -1 * error * 0.01

        self._twist_pub.publish(move_cmd)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(cropped_line_mask))


if __name__ == "__main__":
    autonomous_driver = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down controller")

    cv2.destroyAllWindows()
