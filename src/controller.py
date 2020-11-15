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
        grey_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)[row_max//2:row_max, 0:col_max]

        #mask = cv2.inRange(grey_frame, 240,255)
        mask = cv2.inRange(hsv_frame, (0,0,80), (10,10,90))
        M = cv2.moments(mask)

        out_frame = grey_frame

        error = 0
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            out_frame = cv2.circle(mask, (cX, cY), 40, (50,50,50), -1)
            error = cX - col_max / 2

            # if -10 < error < 10:
            #     error =-10

        move_cmd = Twist()
        move_cmd.linear.x = 0.2 - error * 0.0001
        move_cmd.angular.z = -1 * error * 0.02

        self._twist_pub.publish(move_cmd)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(mask))


if __name__ == "__main__":
    autonomous_driver = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down controller")

    cv2.destroyAllWindows()
