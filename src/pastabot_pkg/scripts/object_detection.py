#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObjectDetection:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
    
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Crop the image
        cropped_img = cv_image[80:720, 220:580]

        # Convert the image to grayscale
        gray = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
        cv.imshow("gray", gray)

        mask = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 3, 3)
        cv.imshow("mask", mask)

        print('###################################-->', cv_image.shape)

        # Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        print("contours: ", contours)

        for cnt in contours:
            cv.polylines(cropped_img, [cnt], True, [255, 0, 0], 1)

        object_detected = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            
            if area > 10 and area < 5000:
                cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
                object_detected.append(cnt)
                print('area-->>>', area)

        print("how many object I detect: ", len(object_detected))
        print(object_detected)

        for cnt in object_detected:
            rect = cv.minAreaRect(cnt)
            (x_center, y_center), (w,h), orientation = rect
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.polylines(cropped_img, [box], True, (255, 0, 0), 1)
            cv.putText(cropped_img, "x: {}".format(round(x_center, 1)) + " y: {}".format(round(y_center, 1)), 
                       (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
            cv.circle(cropped_img, (int(x_center), int(y_center)), 1, (255, 0, 0), thickness=-1)

        # --- Mask creation ---
        # Create mask_special: True for non-black and non-white pixels
        threshold_black = 50
        threshold_white = 200

        # Mask for almost black pixels
        mask_black = np.all(cropped_img <= threshold_black, axis=-1)
        # Mask for almost white pixels
        mask_white = np.all(cropped_img >= threshold_white, axis=-1)

        # Initialize mask_special with True where pixels are not black and not white
        mask_special = ~mask_black & ~mask_white

        # Apply the mask to the image (keeping only the colored pixels)
        cropped_img_masked = np.zeros_like(cropped_img)
        cropped_img_masked[mask_special] = cropped_img[mask_special]

        # --- Show only the masked parts of the image ---
        cv.imshow("Mask Special (No Black or White Pixels)", cropped_img_masked)
        cv.imshow("cropped", cropped_img)
        
        cv.waitKey(1)

if __name__ == '__main__':
    object_detection = ObjectDetection() 
    rospy.init_node('object_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()