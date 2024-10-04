#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from position_tracker.srv import GetPosition, GetPositionResponse

class ObjectDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.server = rospy.Service("/get_position", GetPosition, self.handle_get_position)
    
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
    
            
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        min_red = np.array([0,20,160])
        max_red = np.array([5, 120, 226])

        mask_r = cv.inRange(hsv, min_red, max_red)
        mask = cv.adaptiveThreshold(mask_r, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 3, 3)
        cv.imshow("mask", mask)

        # find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        #print("contours: ", contours)

        for cnt in contours:
            cv.polylines(cv_image, [cnt], True, [255, 0, 0], 1)

        object_detected = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 20:
                cnt = cv.approxPolyDP(cnt, 0.03*cv.arcLength(cnt, True), True)
                object_detected.append(cnt)
        
        #print("how many object I detect: ", len(object_detected))
        #print(object_detected)
                
        x_pxl_center = 319
        y_pxl_center = 240
        pxl_mm_conversion = 27/20
        x0 = 1000
        y0 = 0
        z0 = 500
