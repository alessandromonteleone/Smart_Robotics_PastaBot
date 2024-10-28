#!/usr/bin/env python3

import os
import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def draw_line_with_points(image, start_point, end_point, distance):
    
    # Disegna i cerchi per i punti di inizio e fine
    cv.circle(image, (int(start_point[0]), int(start_point[1])), 5, (0, 0, 255), thickness=-1)
    cv.circle(image, (int(end_point[0]), int(end_point[1])), 5, (0, 255, 0), thickness=-1)

    cv.line(image, 
             (int(start_point[0]), int(start_point[1])), 
             (int(end_point[0]), int(end_point[1])), 
             (255, 255, 255),
             thickness=2)  

    mid_x = int((start_point[0] + end_point[0]) / 2)
    mid_y = int((start_point[1] + end_point[1]) / 2)
    
    cv.putText(image, 
               'start_point', 
               (int(start_point[0])+10, int(start_point[1])),
               cv.FONT_HERSHEY_SIMPLEX, 
               0.7,  
               (0, 0, 255),  
               1)

    cv.putText(image, 
            'end_point', 
            (int(end_point[0])+10, int(end_point[1])),
            cv.FONT_HERSHEY_SIMPLEX, 
            0.7,  # Dimensione del font
            (0, 255, 0),  
            1)  # Spessore del font

    cv.putText(image, 
               distance, 
               (mid_x-80, mid_y), 
               cv.FONT_HERSHEY_SIMPLEX, 
               0.7,
               (255, 255, 255), 
               1)


#All the coords are in the format: (x,y) w.r.t their reference frame
#Note that world reference frame is rotated by 90 degres w.r.t camera frame
class ObjectDetection:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()

        self.push_point_pub = rospy.Publisher("box/push_point", Point, queue_size=10)
        self.distance_pub = rospy.Publisher("box/push_distance", Float32, queue_size=10)

        self.start_end_points = {'start':None, 'end':None}
        self.threshold_wait = 15
        self.counter_position = 0
        self.push_point = None
        self.pixel_tollerence = 3
        self.homography_matrix = np.load(os.getcwd()+'/src/pastabot_pkg/scripts/homography_matrix.npy')
        
        self.first = True

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            if self.first:
                self.first = False
                print("first frame")
                return
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
                #distance along camera y, real x
                distance = self.start_end_points['end'][1] - self.start_end_points['start'][1]
                
                
                start_point_transformed = cv.perspectiveTransform(
                np.array(self.start_end_points['start']).reshape(1,1,2), self.homography_matrix).reshape(2)

                end_point_transformed = cv.perspectiveTransform(
                np.array(self.start_end_points['end']).reshape(1,1,2), self.homography_matrix).reshape(2)

                real_distance = end_point_transformed - start_point_transformed
                distance_norm = np.linalg.norm(real_distance)
                
                if self.push_point:
                    point_msg = Point()
                    point_msg.x = self.push_point[0]
                    point_msg.y = self.push_point[1]
                    point_msg.z = 0.0
                
                # Pubblica la posizione
                self.push_point_pub.publish(point_msg)
                
                if self.counter_position == self.threshold_wait:
                    # Pubblica la distanza
                    self.distance_pub.publish(distance_norm)

                print("Pixel distance: ", abs(distance))
                print(f"Real distance: {real_distance}")
                print(f"Distanza in norma:{distance_norm} ")
                print(f"Matrice {self.homography_matrix}")

                draw_line_with_points(cropped_img, 
                              self.start_end_points['start'], 
                              self.start_end_points['end'],
                              f"{distance_norm:.2f}m")

            cv.circle(mask_black, (int(self.push_point[0]), int(self.push_point[1])), 1, (0, 255, 0), thickness=-1)
            cv.circle(cropped_img, (int(self.push_point[0]), int(self.push_point[1])), 1, (0, 255, 0), thickness=-1)
        
        # --- Show only the masked parts of the image ---
        #cv.imshow("Box frontal face", mask_black)
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
