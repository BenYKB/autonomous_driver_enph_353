import cv2
import imutils
import numpy as np
import pytesseract
import matplotlib.pyplot as plt

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
from tensorflow.python.keras.backend import set_session

import tensorflow as tf
import os

alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
char_lookup = dict(zip([a for a in alphabet],[i for i in range(len(alphabet))]))
char_inverse_lookup = dict(map(reversed, char_lookup.items()))


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


img = cv2.imread('/home/fizzer/Desktop/P2.png',cv2.IMREAD_COLOR)
hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#cv2.imshow('hsv', hsv_frame)
shape = (1280, 720)

normal_lower_hsv = np.array([0,0,86])
normal_upper_hsv = np.array([141,35,125])

darker_lower_hsv = np.array([0,0,148])
darker_upper_hsv = np.array([186,18,202])

img_mask_1 = cv2.inRange(hsv_frame, normal_lower_hsv, normal_upper_hsv)
img_mask_1 = cv2.resize(img_mask_1, shape )

img_mask_2 = cv2.inRange(hsv_frame, darker_lower_hsv, darker_upper_hsv)
img_mask_2 = cv2.resize(img_mask_2, shape )

img_mask = img_mask_1 + img_mask_2

img = cv2.resize(img, shape)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

gray = cv2.bilateralFilter(img_mask, 25, 300, 300)
cv2.imshow('mask', gray)
edged = cv2.Canny(gray, 30, 200)
#cv2.imshow('edges', edged )
contours = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)
contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
screenCnt1 = None

for c in contours:
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.1 * peri, True)
    #cv2.drawContours(gray, [approx], -1, (255, 255, 255), 3)
    #print(len(approx))
    if len(approx) == 4:
        if screenCnt1 is None:
            screenCnt1 = approx
            break

if screenCnt1 is None:
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    screenCnt1 = find_order_pts(screenCnt1)
    cv2.imshow('edges', edged )
    cv2.imshow('mask', gray)

    detected = deskew(img_gray, screenCnt1, [[0,0],[0,1800],[600,1800],[600,0]], (600,1800))

    location = detected[:1800-298-260,:]
    plate = detected[1800-298-260:1800-260,:]

    plate_letters = [plate[50:260,30:170],plate[50:260,130:270], plate[50:260,330:470], plate[50:260,430:570]]
    location_letters = [location[650:1100,0:300],location[650:1100,300:]]

    detected= cv2.resize(detected,(100,150))
    #cv2.imshow('detected', detected)

    try:
        cv2.imshow('Location',location)
        cv2.imshow('Plate', plate)
        pass
    except Exception as e:
        print(e)


    model_shape = (100, 150)
    letters = []


    try:
        for j in range(2):
            letter = location_letters[j]
            letter = cv2.resize(letter, model_shape)
            cv2.imshow('location'+str(j), letter)
            letters.append(letter)

        for j in range(4):
            letter = plate_letters[j]
            letter = cv2.resize(letter, model_shape)
            cv2.imshow('plate' + str(j), letter)
            letters.append(letter)
    except Exception as e:
        print(e)


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
            print(prediction)
            output = output + prediction

            # confidence = something later if we encounter the same plate twice
    print(output)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
