#!/usr/bin/env python

import cv2
import re
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
from tensorflow.python.keras.models import load_model

import tensorflow as tf




alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
char_lookup = dict(zip([a for a in alphabet],[i for i in range(len(alphabet))]))
char_inverse_lookup = dict(map(reversed, char_lookup.items()))

class license_plate_detection():
    def __init__(self):
        rospy.init_node('license_plate_detection', anonymous=True)

        self.sess = tf.keras.backend.get_session()
        self.graph = tf.compat.v1.get_default_graph()

        self.model = load_model(os.path.dirname(os.path.abspath(__file__)) +'/model2')
        self.model._make_predict_function()
        #self.model.summary()

        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback=self._image_callback, queue_size=1)
        self._license_pub = rospy.Publisher('/internal_plate_msg', String, queue_size=1)


    def _image_callback(self, image):
        try:
            img = self._bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            raise e
            return

        letters, confidence = parse_image(img)
        if letters is not None and confidence is not None and len(letters) == 6:
            location, plate =self._get_loc_plate(letters)
            if location is not None:
                self._send_license_plate(location, plate, confidence)


    def _get_loc_plate(self, letters):
        output= ''
        for letter in letters:
            #letter = cv2.imread('/home/fizzer/ros_ws/src/autonomous_driver_enph_353/src/test _images/plate_BF17.png', cv2.IMREAD_GRAYSCALE)[5:,:]
            #letter_reshape = letter.reshape(letter.shape[0], letter.shape[1], 1)
            #letter = cv2.cvtColor(letter, cv2.COLOR_BGR2RGB)
            letter = letter*1./255
            img_aug = np.expand_dims(letter, axis=0)
            #print(img_aug.dtype)
            #print(img_aug.shape)
            with self.graph.as_default():
                tf.keras.backend.set_session(self.sess)
                y_predict = self.model.predict(img_aug)[0]
                #y_predict = self.CNN.predict(letter)
                prediction = char_inverse_lookup[np.argmax(y_predict)]
                output = output + prediction
                #print(prediction)
        #print(output)
        if is_valid(output):
            print('valid output loc plate')
            print(output)
            #print(output)
            return output[1], output[2:]
        else:
            return None, None


    def _send_license_plate(self, loc, plate, confidence):
        print("info")
        print(loc)
        print(plate)
        print(confidence)
        output = ','.join([loc, plate[0], plate[1], plate[2], plate[3], str(confidence)])
        print("sending output" + str(output))
        #print('6,B,A,0,2,1000.2')
        self._license_pub.publish(String(output))
        #self._license_pub.publish(output)


def deskew(img, pts1, pts2, shape):
    pts1 = np.float32(pts1)
    pts2 = np.float32(pts2)

    M, mask = cv2.findHomography(pts1,pts2)
    M = cv2.getPerspectiveTransform(pts1,pts2)

    dst = cv2.warpPerspective(img,M,shape)

    return dst

def find_order_pts(points):
    points = points[:,0,:]
    #points array(array(x,y))
    e1 = np.mean(points[:,0])
    e2 = np.mean(points[:,1])

    low_low = None
    low_high = None
    high_low = None
    high_high = None

    for i in range(4):
        p = points[i]
        if p[0] > e1:
            if p[1] > e2:
                high_high = p
            else:
                high_low = p
        else:
            if p[1] > e2:
                low_high = p
            else:
                low_low = p

    if low_low is None or low_high is None or high_low is None or high_high is None:
        return points
    else:
        return np.array([low_low,low_high,high_high,high_low])

def parse_image(img):
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv', hsv_frame)
    shape = (1280, 720)

    normal_lower_hsv = np.array([0,0,86])
    normal_upper_hsv = np.array([141,35,125])

    darker_lower_hsv = np.array([0,0,148])
    darker_upper_hsv = np.array([186,18,206])

    img_mask_1 = cv2.inRange(hsv_frame, normal_lower_hsv, normal_upper_hsv)
    img_mask_1 = cv2.resize(img_mask_1, shape )

    img_mask_2 = cv2.inRange(hsv_frame, darker_lower_hsv, darker_upper_hsv)
    img_mask_2 = cv2.resize(img_mask_2, shape )

    img_mask = img_mask_1 + img_mask_2

    img = cv2.resize(img, shape)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #gray = cv2.bilateralFilter(img_mask, 25, 100, 100) #300,300
    gray = cv2.GaussianBlur(img_mask,(21,21),0)
    t, gray = cv2.threshold(gray, 127,255,cv2.THRESH_BINARY)

    #print(gray.shape)
    #cv2.imshow('mask', gray)

    edged = cv2.Canny(gray, 30, 200)
    #cv2.imshow('edges', edged )


    im2, contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #print(contours)

    #cv2.drawContours(img, contours, -1, (0, 255, 0), 3)

    #cv2.imshow('m',img)
    #cv2.waitKey(0)

    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
    screenCnt1 = None
    confidence = None

    for c in contours:

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.1 * peri, True)
        cv2.drawContours(img, [approx], -1, (255, 255, 255), 3)
        #print(len(approx))
        if len(approx) == 4:
            if screenCnt1 is None:
                screenCnt1 = approx
                confidence = cv2.contourArea(c)
                break

    if screenCnt1 is None:
        return None, None
    else:
        screenCnt1 = find_order_pts(screenCnt1)
        #cv2.imshow('edges', edged )
        #cv2.imshow('mask', gray)

        if 120 > screenCnt1[1,0]:
            if 50 > screenCnt1[1,0]:
                confidence = confidence * 0.6
            else:
                confidence = confidence * 0.7

        if screenCnt1[2,0] > 1280-120:
            if screenCnt1[2,0] > 1280-50:
                confidence = confidence * 0.6
            else:
                confidence = confidence * 0.7

        detected = deskew(img, screenCnt1, [[0,0],[0,1800],[600,1800],[600,0]], (600,1800))

        location = detected[:1800-298-260,:]
        plate = detected[1800-298-260:1800-260,:]

        plate_letters = [plate[50:260,30:170],plate[50:260,130:270], plate[50:260,330:470], plate[50:260,430:570]]
        location_letters = [location[650:1100,0:300],location[650:1100,300:]]

        letters = []
        model_shape = ( 100, 150)

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
        except Exception as e:
            print(e)
            return None, None

        return letters, confidence

def is_valid(string):
    p = re.compile("^[P][0-9][A-Z][A-Z][0-9][0-9]$")

    return True if p.match(string) else False

    # #return True
    # if not string[0]=='P':
    #     return False
    # elif not string[2:4].isalpha():
    #     return False
    # elif not string[1].isnumeric() or not string[4:].isnumeric():
    #     return False
    # else:
    #     return True


if __name__ == "__main__":
    detection = license_plate_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down controller")

    cv2.destroyAllWindows()
