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
    CROSSWALK_WATCH = 5
    CROSSING = 6
    TO_INNER = 7
    INNER_DRIVE = 8

START_UP_WAIT_TIME = 4
NUM_ROWS = 720
NUM_COLUMNS = 1080
CROSSING_TIME = 10
lower_crosswalk_r1 = 3*NUM_ROWS//4 
lower_crosswalk_r2 = NUM_ROWS
lower_crosswalk_c1 = NUM_COLUMNS//4
lower_crosswalk_c2 = 3*NUM_COLUMNS//4

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
        self.time_since_crossing_started = 0
        self.past_error = 0
        self.past_saw_left = False
        self.left_following = True
        rospy.Timer(rospy.Duration(1), self._on_timer)

    def _on_timer(self, time):
        if not self.initialized:
            return

        self.seconds += 1
        
        if self.seconds == 1:
            self.start_timer()
            self.state = States.STARTING
        
        # if self.seconds == 60*4-2:
        #     self.stop_timer()
        #     self.state = States.STOP

        # if self.seconds >= 20:
        #     self.state = States.STOP

        if self.state == States.STARTING and self.seconds >= START_UP_WAIT_TIME:
            self.state = States.FOLLOWING
        
        if self.state == States.CROSSING:
            self.time_since_crossing_started += 1
            
            if self.time_since_crossing_started > CROSSING_TIME:
                self.state = States.FOLLOWING
                

        print(self.state)

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
        
        print(rospy.get_time())
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        out_frame = cv_image
        move_cmd = Twist()

        # print('shape:')
        # print(cv_image.shape) --> 720, 1080, 3
        
        if self.state == States.FOLLOWING or self.state == States.STARTING or self.state == States.CROSSING:
            LEFT_FOLLOWING = self.left_following
            print(self.left_following)

            row_max, col_max, channels = cv_image.shape
            hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            road_mask = cv2.inRange(hsv_frame, (0,0,80), (10,10,90))
            road_mask_mid = road_mask[:, 1*(col_max//3):2*(col_max//3)]

            line_mask = cv2.inRange(hsv_frame, (0,0,248), (2,2,255))
            cross_walk_red_mask = cv2.inRange(hsv_frame, (0,250,250), (2,255,255))
            car_mask = cv2.inRange(hsv_frame, (116,125, 197),(124,133,203))  + cv2.inRange(hsv_frame, (115,245,95),(125,260,105))# (197,125, 116),(203,133,124))
            truck_mask = cv2.inRange(hsv_frame, (0,0,12),(1,1,20)) #(45,0,0),(220,1,1))

#            out_frame = hsv_frame
            out_frame = car_mask//2 + truck_mask
            out_frame = cv2.cvtColor(out_frame, cv2.COLOR_GRAY2BGR)

            truckness = np.sum(truck_mask)
            cv2.putText(out_frame, str(np.sum(car_mask[340:,700:])), (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),2,cv2.LINE_AA)

            PASSING_INNER_CAR = False
            #if 

            USE_CENTER = False
            if np.sum(road_mask_mid[500:]) > 0.7*255*(row_max-500)*road_mask_mid.shape[1] and np.sum(road_mask_mid[360:500]) > 0.6*255*(500-360)*road_mask_mid.shape[1]:
                USE_CENTER = True
            
            line_col_start = 1*(col_max//3)
            line_row_start = -1*(row_max//4)
            cropped_line_mask_right = line_mask[line_row_start:, line_col_start:]
            cropped_line_mask_left = line_mask[line_row_start:, :-line_col_start]

            if USE_CENTER:
                M_road = cv2.moments(road_mask_mid)
            else:
                M_road = cv2.moments(road_mask)
            M_line_R = cv2.moments(cropped_line_mask_right)
            M_line_L = cv2.moments(cropped_line_mask_left)

            alpha_line = 0.27 #0.2 for outer

            lower_red_mask = cross_walk_red_mask[lower_crosswalk_r1:lower_crosswalk_r2,lower_crosswalk_c1:lower_crosswalk_c2]

            #out_frame = hsv_frame

            error = 0
            if M_road["m00"] != 0:
                cX = int(M_road["m10"] / M_road["m00"])
                cY = int(M_road["m01"] / M_road["m00"])

                if USE_CENTER:
                    error = cX - (col_max//6)
                else:
                    error = cX - col_max / 2

                if M_line_R["m00"] != 0 and self.state != States.STARTING and not LEFT_FOLLOWING:
                    cX = int(M_line_R["m10"] / M_line_R["m00"])
                    cY = int(M_line_R["m01"] / M_line_R["m00"])

                    error = (1 - alpha_line) * error + alpha_line * (cX - (4 * line_col_start) / 5)

                if M_line_L["m00"] != 0 and self.state != States.STARTING and LEFT_FOLLOWING:
                    cX = int(M_line_L["m10"] / M_line_L["m00"])
                    cY = int(M_line_L["m01"] / M_line_L["m00"])
                    print(cX)

                    error = (1 - alpha_line) * error + -1 * alpha_line * (cX - (1.5 * line_col_start) / 5)*1.3

                    #out_frame = cv2.circle(out_frame, (cX,cY), 5,(255,0,0))
                    self.past_saw_left = True
                elif M_line_L["m00"] == 0 and self.state != States.STARTING and LEFT_FOLLOWING:
                    if self.past_saw_left:
                        error = (1 - alpha_line) * error + -1 * alpha_line * (500 - (1.5 * line_col_start) / 5)
                        print("lost left")

                        if np.sum(road_mask[-row_max//5:, :col_max//3]) > 255*0.5*row_max//5*col_max//3:
                            self.past_saw_left = True
                        else:
                            self.past_saw_left = False
                    else:
                        self.past_saw_left = False

            low_pass_K = 0.2
            error = (1-low_pass_K) * error + low_pass_K * self.past_error
            move_cmd.linear.x = 0.1 - error * 0.0001
            move_cmd.angular.z = -1 * error * 0.01 if self.state != States.STARTING else -1 * error * 0.01 + 0.1

            self.past_error = error

            if np.sum(car_mask[340:,700:]) > 0.8*255*(1280-790)*(100):
                self.left_following = False
            if np.sum(truck_mask) > 200000:
                move_cmd = Twist()

            if np.sum(lower_red_mask) > lower_red_mask.shape[0] * lower_red_mask.shape[1] * 255//4 and self.state != States.CROSSING:
                move_cmd = Twist()
                self.state = States.CROSSWALK_WATCH

            # if np.sum(cropped_line_mask[200:, 2*1080//5:3*1080//5]) >= 0.5*200*1080//5*255:
            #     #self.state = States.STOP
            #     move_cmd = Twist()
            #     print("red detected; stopping")
            #     print(cropped_line_mask[200:, 2*1080//5:3*1080//5])

        elif self.state == States.CROSSWALK_WATCH:
            hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            white_mask = cv2.inRange(hsv_frame, (0,0,253), (2,2,255))
            M_white = cv2.moments(white_mask)
            

            # looks like crossing centroid is a sufficient indicator of safety
            if M_white["m00"] != 0:
                cX = int(M_white["m10"] / M_white["m00"])
                purple_mask = cv2.inRange(hsv_frame[:, cX-10:cX+10], (99,40,40), (106,100,160))

                if np.sum(purple_mask) > 255 * 3 * 17:
                    self.state = States.CROSSING

                #out_frame = cv2.cvtColor(purple_mask, cv2.COLOR_GRAY2BGR)
                #cY = int(M_white["m01"] / M_white["m00"])
            else:
                self.state == States.CROSSING

            if self.state == States.CROSSING:
                self.time_since_crossing_started = 0
                
            move_cmd = Twist()
            
        elif self.state == States.STOP:
            move_cmd.linear.x = 0
        elif self.state == States.SPIN:
            move_cmd.angular.z = np.pi/2
        else:
            raise Exception('Invalid State')

        self._twist_pub.publish(move_cmd)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(out_frame, "bgr8"))
        self.image_count += 1


if __name__ == "__main__":
    autonomous_driver = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down controller")

    cv2.destroyAllWindows()
