#!/usr/bin/env python

import cv2
#import csv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os
#import pyqrcode
#import random
#import string
import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import imutils

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
from tensorflow.python.keras.backend import set_session

import tensorflow as tf

#sess = tf.Session()
#graph = tf.get_default_graph()

#CNN._make_predict_function()
#CNN.summary()

team_id ='12345678'
password = '12345678'

alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
char_lookup = dict(zip([a for a in alphabet],[i for i in range(len(alphabet))]))
char_inverse_lookup = dict(map(reversed, char_lookup.items()))

class license_plate_detection():
    def __init__(self):
        rospy.init_node('license_plate_detection', anonymous=True)

        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback=self._image_callback, queue_size=1)
        self._license_pub = rospy.Publisher('/license_plate', String, queue_size=1)


    def _image_callback(self, image):
        try:
            img = self._bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            raise e
            return

        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        shape = (1280, 720)
        img_mask = cv2.inRange(hsv_frame, (0,0,90), (10,10,110))
        img_mask = cv2.resize(img_mask, shape )
        img = cv2.resize(img, shape)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        gray = cv2.bilateralFilter(img_mask, 13, 15, 15)

        edged = cv2.Canny(gray, 30, 200)
        contours = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
        screenCnt = None

        for c in contours:

            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.018 * peri, True)

            if len(approx) == 4:
                screenCnt = approx
                break

        if screenCnt is None:
            return
        else:
             cv2.drawContours(img, [screenCnt], -1, (0, 0, 255), 3)
             mask = np.zeros(gray.shape,np.uint8)
             new_image = cv2.drawContours(mask,[screenCnt],0,255,-1,)
             new_image = cv2.bitwise_and(img,img,mask=mask)

             (x, y) = np.where(mask == 255)
             (topx, topy) = (np.min(x), np.min(y))
             (bottomx, bottomy) = (np.max(x), np.max(y))
             height = bottomx-topx
             location = img_gray[topx+height/2:bottomx+1, topy:bottomy+1]
             plate = img_gray[bottomx:bottomx+height/4, topy:bottomy+1]
             plate_shape = plate.shape
             plate_increment = plate_shape[1]/24
             plate_coords = [[1,7], [5,11], [13,19], [17,23]]
             location_increment =  plate_shape[1]/2
             plate_letters = [plate[:,plate_increment*plate_coords[i][0]:plate_increment*plate_coords[i][1]] for i in range(4)]
             location_letters = [location[:,location_increment*i:location_increment*(i+1)] for i in range(5)]

             try:
                 #cv2.imshow('Location',location)
                 #cv2.imshow('Plate', plate)
                 pass
             except Exception as e:
                 pass


             model_shape = (100, 150)
             letters = []

             try:
                 for j in range(2):
                     letter = location_letters[j]
                     letter = cv2.resize(letter, model_shape)
                     #cv2.imshow('location'+str(j), letter)
                     letters.append(letter)

                 for j in range(4):
                     letter = plate_letters[j]
                     letter = cv2.resize(letter, model_shape)
                     #cv2.imshow('plate' + str(j), letter)
                     letters.append(letter)

                 location, plate = self._get_loc_plate(letters)
                 if location is not None:
                     self._send_license_plate(location, plate)
             except Exception as e:
                 pass

             cv2.destroyAllWindows()



    def _get_loc_plate(self, letters):
        output = ''
        CNN = tf.keras.models.load_model(os.path.dirname(os.path.abspath(__file__)) +'/my_model')
        CNN._make_predict_function()
        for letter in letters:
            #letter = cv2.imread('/home/fizzer/ros_ws/src/autonomous_driver_enph_353/src/test _images/plate_BF17.png', cv2.IMREAD_GRAYSCALE)[5:,:]
            letter_reshape = letter.reshape(letter.shape[0], letter.shape[1], 1)
            img_aug = np.expand_dims(letter_reshape, axis=0)
            #print(img_aug.shape)
            if CNN is not None:
                y_predict = CNN.predict(img_aug)[0]
                #y_predict = self.CNN.predict(letter)
                prediction = char_inverse_lookup[np.argmax(y_predict)]
                if not cv2.imwrite('/home/fizzer/ros_ws/src/autonomous_driver_enph_353/src/test_images/' + prediction + '.png', letter):
                    raise Exception("Could not write image")
                output = output + prediction
                # confidence = something later if we encounter the same plate twice
        if output[0] == 'P':
            print(output)
            return output[1], output[2:]
        else:
            return None, None

    def _send_license_plate(self, loc, plate):
        output = ', '.join([team_id, password, loc, plate])
        self._license_pub.publish(output)


if __name__ == "__main__":
    detection = license_plate_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down controller")

    cv2.destroyAllWindows()
