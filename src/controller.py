#!/usr/bin/env python

from enum import Enum
import cv2
#import csv
import numpy as np
from sensor_msgs.msg import Image
#import os
#import pyqrcode
#import random
from std_msgs.msg import String
from std_msgs.msg import Time
import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


# Code used for autonomous control of rover

class States(Enum):
    STARTING = 1
    FOLLOWING = 2
    SPIN = 3
    STOP = 4

START_UP_WAIT_TIME = 4

class controller():
    def __init__(self):
        self.initialized = False

        rospy.init_node('controller', anonymous=True)
        self._image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback=self._image_callback, queue_size=1)
        self._clock_sub = rospy.Subscriber('/clock', Time, callback=self._time_callback, queue_size=1)
        
        self._twist_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self._image_pub = rospy.Publisher('/R1/debug_image', Image, queue_size=1)
        self._licence_pub = rospy.Publisher('/license_plate', String, queue_size=1)  # note no slash in the source code

        self._bridge = CvBridge()
        self.image_count = 0
        self.state = States.STOP
        self.seconds = 0
        rospy.Timer(rospy.Duration(1), self._on_timer)

    def _on_timer(self, time):
        if not self.initialized:
            return

        self.seconds += 1
        
        if self.seconds == 1:
            self.start_timer()
            self.state = States.STARTING
        
        if self.seconds == 60*4:
            self.stop_timer()
            self.state = States.STOP

        # if self.seconds >= 20:
        #     self.state = States.STOP

        if self.state == States.STARTING and self.seconds >= START_UP_WAIT_TIME:
            self.state = States.FOLLOWING

    def _time_callback(self, time):
        if not self.initialized and rospy.get_time() > 1:
            self.initialized = True

    def start_timer(self):
        self._licence_pub.publish(str('team1,xxxx,0,AA00'))
    
    def stop_timer(self):
        self._licence_pub.publish(str('team1,xxxx,-1,AA00'))


    def _image_callback(self, image):
        if not self.initialized:
            return
        
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            raise e
        out_frame = cv_image
        move_cmd = Twist()
        
        if self.state == States.STARTING:    
            move_cmd.linear.x = 0.15
            move_cmd.angular.z = 0.3
        elif self.state == States.FOLLOWING:
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

                    error = (1 - alpha_line) * error + alpha_line * (cX - (4 * line_col_start) / 5)

            move_cmd.linear.x = 0.1 - error * 0.0001
            move_cmd.angular.z = -1 * error * 0.01
        elif self.state == States.STOP:
            move_cmd.linear.x = 0
        elif self.state == States.SPIN:
            move_cmd.angular.z = np.pi/2
        else:
            raise Exception('Invalid State')

        self._twist_pub.publish(move_cmd)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(out_frame))
        self.image_count += 1


if __name__ == "__main__":
    autonomous_driver = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down controller")

    cv2.destroyAllWindows()
