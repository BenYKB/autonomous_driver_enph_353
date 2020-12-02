#!/usr/bin/env python

from enum import Enum
import cv2
#import csv
import numpy as np
from sensor_msgs.msg import Image
#import os
#import pyqrcode
#import random
from std_msgs.msg import Time
from std_msgs.msg import String
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

START_UP_WAIT_TIME = 7 # 8 #TODO: Change back after debugging!
NUM_ROWS = 720
NUM_COLUMNS = 1080
CROSSING_TIME = 10
CROSS_DELAY = 2
TO_INNER_TIMEOUT = 86 #86 #TODO: Change back after debugging!
lower_crosswalk_r1 = 3*NUM_ROWS//4 
lower_crosswalk_r2 = NUM_ROWS
lower_crosswalk_c1 = NUM_COLUMNS//4
lower_crosswalk_c2 = 3*NUM_COLUMNS//4

inner_car_r1 = 340
inner_car_r2 = 720
inner_car_c1 = 700
inner_car_c2 = 1280


class controller():
    def __init__(self):
        self.initialized = False

        rospy.init_node('controller', anonymous=True)
        self._image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback=self._image_callback, queue_size=1)
        self._clock_sub = rospy.Subscriber('/clock', Time, callback=self._time_callback, queue_size=1)
        self._internal_plate_sub = rospy.Subscriber('/internal_plate_msg', String, callback=self._on_plate, queue_size=10)
        
        self._twist_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self._image_pub = rospy.Publisher('/R1/debug_image', Image, queue_size=1)
        self._licence_pub = rospy.Publisher('/license_plate', String, queue_size=1)  # note no slash in the source code
        self._internal_plate_pub = rospy.Publisher('/internal_plate_msg', String, queue_size=10)

        self._bridge = CvBridge()
        self.image_count = 0
        self.state = States.STOP
        self.previous_state = States.STOP

        self.seconds = 0
        self.time_since_crossing_started = 0
        self.past_error = 0
        self.past_saw_left = False
        self.is_done_outside = False

        self.start_time = None

        self.plates = {
            1: ['A','A','0','0', -1.0],
            2: ['A','A','0','0', -1.0],
            3: ['A','A','0','0', -1.0],
            4: ['A','A','0','0', -1.0],
            5: ['A','A','0','0', -1.0],
            6: ['A','A','0','0', -1.0],
            7: ['A','A','0','0', -1.0],
            8: ['A','A','0','0', -1.0],
        }

        rospy.Timer(rospy.Duration(1), self._on_timer)

    def _on_plate(self, plate_string):
        plate_string = plate_string.data
        plate_list = plate_string.split(',')

        if len(plate_list) != 6:
            print("message invalid length:")
            print(plate_string)

        try:
            P = int(plate_list[0])
            C = float(plate_list[-1])
        except ValueError:
            print("message invalid types:")
            print(plate_string)
            return
        
        if not (0 < P < 9):
            print("P out of range")
            print(P)
            return

        plate_value =  plate_list[1:5]
        plate_value.append(C)

        current_C = self.plates[P][-1]

        print("old confidence")
        print(current_C)
        print("new confidence")
        print(C)

        if C > current_C and self.seconds > 1:
            print("updated plate at")
            print(P)
            self.plates[P] = plate_value
            
            msg = 'team1,xxxx,'+str(P)+','+''.join(plate_value[0:4])
            #ros_msg = String(msg)
            self._licence_pub.publish(str(msg))
            print("sending msg")
            print(msg)
            # if P == 1:
            #     self.is_done_outside = True

            #TODO: Stop if got both inner plates

    def _on_timer(self, time):
        if not self.initialized:
            return

        self.seconds += 1
        
        #TODO: remove if not debugging
        #self._internal_plate_pub.publish(String('5,A,A,2,2,0.9'))

        time_r = rospy.get_time()
        time_from_start = time_r
        if time_r != 0:
            if not self.start_time:
                self.start_time = time_r
            time_from_start = time_r - self.start_time
            print(time_from_start)
        
        if TO_INNER_TIMEOUT < time_from_start:
            #TODO: detect if enough plates have been detected
            self.is_done_outside = True
        
        if self.seconds == 1:
            self.start_timer()
            self.state = States.STARTING
        
        if self.seconds >= 150:
            self.stop_timer()
            self.state = States.STOP

        # if self.seconds >= 20:
        #     self.state = States.STOP

        if self.state == States.STARTING and self.seconds >= START_UP_WAIT_TIME:
            self.state = States.FOLLOWING
        
        if self.state == States.CROSSING:
            self.time_since_crossing_started += 1
            
            if self.time_since_crossing_started > CROSSING_TIME:
                if self.is_done_outside:
                    self.state = States.TO_INNER
                else:
                    self.state = States.FOLLOWING

        if self.is_done_outside and self.state == States.FOLLOWING:
            self.state = States.TO_INNER
                

        #print(self.state)

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
        
        #print(rospy.get_time())
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        out_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        move_cmd = Twist()

        # print('shape:')
        # print(cv_image.shape) --> 720, 1080, 3


        if self.state == States.FOLLOWING or self.state == States.STARTING or self.state == States.CROSSING or self.state == States.TO_INNER or self.state == States.INNER_DRIVE:
            row_max, col_max, channels = cv_image.shape
            hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # ROAD
            road_mask = cv2.inRange(hsv_frame, (0,0,80), (10,10,90))
            road_mask_mid = road_mask[:, 1*(col_max//3):2*(col_max//3)]

            #ROAD LINES
            line_col_start = 1*(col_max//3)
            line_row_start = -1*(row_max//4)
            
            line_mask = cv2.inRange(hsv_frame[line_row_start:,:], (0,0,248), (2,2,255))
            
            cropped_line_mask_right = line_mask[:, line_col_start:]
            cropped_line_mask_left = line_mask[:, :-line_col_start]

            #CROSSWALK
            cross_walk_red_mask = cv2.inRange(hsv_frame[lower_crosswalk_r1:lower_crosswalk_r2,lower_crosswalk_c1:lower_crosswalk_c2], (0,250,250), (2,255,255))
 
            lower_red_mask = cross_walk_red_mask
            
            #CAR DETECTION
            if self.state == States.TO_INNER or self.state == States.INNER_DRIVE:
                car_mask = cv2.inRange(hsv_frame, (116,125, 197),(124,133,203)) + cv2.inRange(hsv_frame, (115,245,95),(125,260,105)) # (197,125, 116),(203,133,124))
                transition_car_mask = car_mask[inner_car_r1:inner_car_r2, inner_car_c1:inner_car_c2]
                width = 200
                car_ML = np.sum(car_mask[435:550,640-width:640]) / (255.0 * width * 115) 
                car_MR = np.sum(car_mask[435:550,640:640+width]) / (255.0 * width * 115) 
                car_SL = np.sum(car_mask[560:,:280]) / (255.0 * 160 * 280)
                car_SR = np.sum(car_mask[560:,-280:]) / (255.0 * 160 * 280)
            
            #TRUCK DETECTION (Tires)
            if self.state == States.TO_INNER or self.state == States.INNER_DRIVE:
                truck_mask = cv2.inRange(hsv_frame, (0,0,12),(1,1,20)) #(45,0,0),(220,1,1))

            #SET OUT FRAME
            #out_frame = hsv_frame
            #out_frame = car_mask//2 + truck_mask
            #out_frame = cv2.cvtColor(out_frame, cv2.COLOR_GRAY2BGR)

            # truckness = np.sum(truck_mask)
            # cv2.putText(out_frame, str(np.sum(car_mask[340:,700:])), (300,300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),2,cv2.LINE_AA)

            LEFT_FOLLOWING = False
            if self.state == States.TO_INNER:
                LEFT_FOLLOWING = True

            USE_CENTER = False
            if np.sum(road_mask_mid[500:]) > 0.7*255*(row_max-500)*road_mask_mid.shape[1] and np.sum(road_mask_mid[360:500]) > 0.6*255*(500-360)*road_mask_mid.shape[1]:
                USE_CENTER = True
            if self.state == States.INNER_DRIVE:
                USE_CENTER = False

            if USE_CENTER:
                M_road = cv2.moments(road_mask_mid)
            else:
                M_road = cv2.moments(road_mask)

            if LEFT_FOLLOWING:
                M_line_L = cv2.moments(cropped_line_mask_left)
            else:
                M_line_R = cv2.moments(cropped_line_mask_right)
            
            alpha_line = 0.27 
            #0.2 for outer
            #out_frame = hsv_frame

            error = 0
            if M_road["m00"] != 0:
                cX = int(M_road["m10"] / M_road["m00"])
                cY = int(M_road["m01"] / M_road["m00"])

                if USE_CENTER:
                    error = cX - (col_max//6)
                else:
                    error = cX - col_max / 2

                if not LEFT_FOLLOWING and M_line_R["m00"] != 0 and self.state != States.STARTING:
                    cX = int(M_line_R["m10"] / M_line_R["m00"])
                    cY = int(M_line_R["m01"] / M_line_R["m00"])

                    error = (1 - alpha_line) * error + alpha_line * (cX - (4 * line_col_start) / 5)

                if LEFT_FOLLOWING and M_line_L["m00"] != 0 and self.state != States.STARTING:
                    cX = int(M_line_L["m10"] / M_line_L["m00"])
                    cY = int(M_line_L["m01"] / M_line_L["m00"])
                    #print(cX)

                    error = (1 - alpha_line) * error + -1 * alpha_line * (cX - (1.4 * line_col_start) / 5)*1.0

                    #out_frame = cv2.circle(out_frame, (cX,cY), 5,(255,0,0))
                    self.past_saw_left = True
                elif LEFT_FOLLOWING and M_line_L["m00"] == 0 and self.state != States.STARTING:
                    if self.past_saw_left:
                        error = (1 - alpha_line) * error + -1 * alpha_line * (500 - (1.5 * line_col_start) / 5)
                        #print("lost left")

                        if np.sum(road_mask[-row_max//5:, :col_max//3]) > 255*0.5*row_max//5*col_max//3:
                            self.past_saw_left = True
                        else:
                            self.past_saw_left = False
                    else:
                        self.past_saw_left = False
                
                if self.state == States.TO_INNER or self.state == States.INNER_DRIVE:
                    CAR_MID_THRESHOLD = 0.5
                    CAR_MID_A = 0.1
                    CAR_MID_P = 400
                    CAR_SIDE_THRESHOLD = 0.1
                    CAR_SIDE_SETPOINT = 0.5
                    CAR_SIDE_A = 0.1
                    CAR_SIDE_P = 200

                    # if car_SL > car_SR:
                    #     if car_SL > CAR_SIDE_THRESHOLD:
                    #         error = (1 - CAR_SIDE_A) * error - CAR_SIDE_A * CAR_SIDE_P * (car_SL - CAR_MID_THRESHOLD) 
                    # else:
                    #     if car_SR > CAR_SIDE_THRESHOLD:
                    #         error = (1 - CAR_SIDE_A) * error + CAR_SIDE_A * CAR_SIDE_P * (car_SR - CAR_MID_THRESHOLD) 
                    
                    if car_ML > car_MR:
                        if car_ML > CAR_MID_THRESHOLD:
                            error = (1 - CAR_MID_A) * error + CAR_MID_A * car_ML * CAR_MID_P

                    else:
                        if car_MR > CAR_MID_THRESHOLD:
                            error = (1 - CAR_MID_A) * error - CAR_MID_A * car_MR * CAR_MID_P


            low_pass_K = 0.2
            error = (1-low_pass_K) * error + low_pass_K * self.past_error

            linear_cmd = 0.14 - error * 0.0001
            angular_cmd = -1 * error * 0.01

            if self.state == States.STARTING:
                angular_cmd += 0.1
            elif self.state == States.INNER_DRIVE:
                angular_cmd -= 0.3
            move_cmd.linear.x = linear_cmd if self.previous_state != States.CROSSWALK_WATCH else linear_cmd*0.25
            move_cmd.angular.z = angular_cmd

            self.past_error = error

            # SEEING MIDDLE CAR 
            if self.state == States.TO_INNER and np.sum(transition_car_mask) > 0.5*255*(1280-790)*(100):
                self.state = States.INNER_DRIVE

            #EMERGENCY STOPPING
            if (self.state == States.TO_INNER or self.state == States.INNER_DRIVE) and np.sum(truck_mask) > 200000:
                move_cmd = Twist()

            if np.sum(lower_red_mask) > lower_red_mask.shape[0] * lower_red_mask.shape[1] * 255//5 and self.state != States.CROSSING:
                move_cmd = Twist()
                self.state = States.CROSSWALK_WATCH
                #print("Crosswalk detected")

            if self.state == States.CROSSING and self.time_since_crossing_started <= CROSS_DELAY:
                move_cmd = Twist()

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
                    self.time_since_crossing_started = 0

                #out_frame = cv2.cvtColor(purple_mask, cv2.COLOR_GRAY2BGR)
                #cY = int(M_white["m01"] / M_white["m00"])
            else:
                self.state == States.CROSSING
                self.time_since_crossing_started = 0
                
                
            move_cmd = Twist()
            
        elif self.state == States.STOP:
            move_cmd.linear.x = 0
        elif self.state == States.SPIN:
            move_cmd.angular.z = np.pi/2
        else:
            raise Exception('Invalid State')

        self.previous_state = self.state
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
