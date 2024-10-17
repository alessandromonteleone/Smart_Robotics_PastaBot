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

import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ObjectDetection:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.start_end_points = {'start':None, 'end':None}
        self.threshold_wait = 15
        self.counter_position = 0
        self.push_point = None
        self.pixel_tollerence = 3
    
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Crop the image
        #(192;10) (607;10) (192;789) (607;789) 

        cropped_img = cv_image[10:789+1, 192:607+1]
        #logger.debug(f"cropped_img.shape: {cropped_img.shape}")

        """
        # Convert the image to grayscale
        gray = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
        #cv.imshow("gray", gray)

        mask = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 3, 3)
        #cv.imshow("mask", mask)

        # Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        logger.info("contours: ", contours)

        for cnt in contours:
            cv.polylines(cropped_img, [cnt], True, [255, 0, 0], 1)

        object_detected = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 10 and area < 5000:
                cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
                object_detected.append(cnt)
                logger.debug(f'area-->>> {area}')

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

        """

        # --- Mask creation ---
        threshold_black = 30

        # Mask for almost black pixels
        mask_black = ((cropped_img <= threshold_black) * 255).astype(np.uint8)
        mask_black = cv.cvtColor(mask_black, cv.COLOR_BGR2GRAY)
        
        # Apply the mask to the image (keeping only the colored pixels)
        #face = np.ones_like(cropped_img) * 255
        #face[mask_black] = cropped_img[mask_black]
        contours, _ = cv.findContours(mask_black, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        mask_black = cv.cvtColor(mask_black, cv.COLOR_GRAY2BGR)
        for cnt in contours:
            cv.polylines(cropped_img, [cnt], True, [0, 0, 255], 1)
            cv.polylines(mask_black, [cnt], True, [0, 0, 255], 1)

        objects_detected = []
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 20:
                cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
                if cnt.shape[0] == 4:
                    objects_detected.append(cnt)
                    #print("cnt:", cnt)

        print("#"*10 + " Detected ", len(objects_detected), "objects")

        if len(objects_detected):
            points = objects_detected[0].squeeze(-2)
            idx = np.argsort(points[:,-1])[2:4]
            bottom_side = points[idx, :]
            # print('point', points.shape)
            # print('idx', idx.shape)
            # print('bottom', bottom_side.shape)
            
            if self.push_point is not None:
                prev_push_point = self.push_point
            
            self.push_point = (bottom_side.sum(-2) / 2).tolist()
            print("self.push_point (x, y)" + str(self.push_point))

            if self.start_end_points['start'] is None:
                self.start_end_points['start'] = self.push_point
            elif (abs(prev_push_point[1]-self.push_point[1]) < self.pixel_tollerence 
                  and abs(self.start_end_points['start'][1]-self.push_point[1])> 5):
                self.counter_position+=1
                print("counter:", self.counter_position)
            
            if self.counter_position >= self.threshold_wait:
                self.start_end_points['end'] = self.push_point
                distance = self.start_end_points['end'][1] - self.start_end_points['start'][1]
                print("DISTANCE: ", abs(distance))
            

            cv.circle(mask_black, (int(self.push_point[0]), int(self.push_point[1])), 1, (0, 255, 0), thickness=-1)
            cv.circle(cropped_img, (int(self.push_point[0]), int(self.push_point[1])), 1, (0, 255, 0), thickness=-1)
        
        # --- Show only the masked parts of the image ---
        cv.imshow("Box frontal face", mask_black)
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
