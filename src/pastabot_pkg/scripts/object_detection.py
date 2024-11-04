#!/usr/bin/env python3

import os
import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Point
import logging


## GLOBAL VARIABLES
BOX_SIDES = [0.08, 0.1]
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


## FUNCTIONS
def compute_side_point(push_point: list, dest: str, side_lengths: list):
    # PUSH POINT: middle point of bottom side in real-world coords
    x, y = push_point
    side_x, side_y = side_lengths
    x += side_x/2

    if dest == "left":
        return (x + side_x/2 + 0.025, y + side_y/2 + 0.025)
    elif dest == "right":
        return (x + side_x/2 + 0.025, y - side_y/2 - 0.025)
    else:
        return x, y


def list2d_to_point(list2d) -> Point:
    # 'list2d' is a list or a np.array with 2 components
    point_msg = Point()
    point_msg.x = list2d[0]
    point_msg.y = list2d[1]
    point_msg.z = 0.0
    return point_msg


def imgframe_to_worldframe(img_list2d: list, H: np.array):
    return (
        cv.perspectiveTransform(np.array(img_list2d).reshape(1, 1, 2), H)
        .reshape(2)
        .tolist()
    )


def draw_line_with_points(image, start_point, end_point, side_point, distance):
    # Draw circles for START and END points
    cv.circle(image, (int(start_point[0]), int(start_point[1])), 5, (0, 0, 255), thickness=-1)
    cv.circle(image, (int(end_point[0]), int(end_point[1])), 5, (0, 255, 0), thickness=-1)
    
    if side_point:
        cv.circle(image, (int(side_point[0]), int(side_point[1])), 5, (255, 150, 75), thickness=-1)

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
            0.7,  # Font dimension
            (0, 255, 0),  
            1)    # Font width

    cv.putText(image, 
               distance, 
               (mid_x-80, mid_y), 
               cv.FONT_HERSHEY_SIMPLEX, 
               0.7,
               (255, 255, 255), 
               1)


# NOTE: All the coords are in the format (x,y) w.r.t their reference frame. Consider that world reference frame is rotated by 90 degres w.r.t camera frame
## CLASS
class ObjectDetection:
    def __init__(self):
        self.push_point_pub = rospy.Publisher("box/push_point", Point, queue_size=10)
        self.distance_pub = rospy.Publisher("box/push_distance", Float32, queue_size=10)
        self.initial_point_pub = rospy.Publisher("box/initial_point", Point, queue_size=10)
        self.side_pub = rospy.Publisher("box/side_point", Point, queue_size=10)
        self.dest_pub = rospy.Publisher("box/dest", String, queue_size=10)

        self.bridge_object = CvBridge()
        self.threshold_wait = 10
        
        self.pixel_tolerance = 3
        self.area_threshold = 10
        self.rgb_threshold = [30, 30, 30]
        self.homography_matrix = np.load(os.getcwd() + '/homography_matrix.npy') # TODO: change when in roslaunch

        self.stop_detector_sub = rospy.Subscriber("force_check/stop_detector", Bool, self.stop_detector_callback)
        self.box_type_sub = rospy.Subscriber("box/type_topic", String, self.box_type_callback)
        self.type2dest_mapping = {
            "No Object": "front",  # HACK
            "LIGHT BOX": "front",
            "MEDIUM BOX": "right",
            "HEAVY BOX": "left",
        }

        self.first = True
        self.is_stopped = True
        self.start_end_points = {'start': None, 'end': None}
        self.counter_position = 0
        self.push_point = None
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)


    def start(self):
        self.first = True
        self.is_stopped = False
        self.start_end_points = {'start':None, 'end':None}

        #self.bridge_object = CvBridge()
        self.counter_position = 0
        self.push_point = None
        while not self.is_stopped:
            rospy.sleep(0.001)

    def stop(self):
        # self.image_sub.unregister()
        print("Detector stopped")
        self.is_stopped = True

    def box_type_callback(self, msg):
        box_type = msg.data
        assert box_type in self.type2dest_mapping, f"ERROR, {box_type} not in {self.type2dest_mapping.keys()}"

        if self.push_point is None:
            rospy.logwarn("box type message received, but push_point is None!")
            return
       
        if box_type == "No Object":
            # Stops Detector when box falls from the table
            self.stop() 
        else:
            dest = self.type2dest_mapping[box_type]
            push_list2d_world = imgframe_to_worldframe(self.push_point, self.homography_matrix)
            side_list2d_world = compute_side_point(push_list2d_world, dest, BOX_SIDES)
            side_msg = list2d_to_point(side_list2d_world)
            self.side_pub.publish(side_msg)
            self.dest_pub.publish(dest)
            print(f"ATTENTION: Side point --> {side_list2d_world}, Dest --> {dest}")

    def stop_detector_callback(self, msg: Bool):
        if msg.data:
            print("stopped detector")
            self.stop()

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            if self.first:
                self.first = False
                #print("first frame")
                return
            if self.is_stopped:
                return
        except CvBridgeError as e:
            print(e)
        
        # Crop the image
        #(192;10) (607;10) (192;789) (607;789) 

        cropped_img = cv_image[10:789+1, 192:607+1]
        #logger.debug(f"cropped_img.shape: {cropped_img.shape}")
        
        # --- Mask creation ---
        threshold_black = np.array(self.rgb_threshold).reshape(1,1,3)

        # Mask for almost black pixels
        mask_black = ((cropped_img <= threshold_black)* 255).astype(np.uint8)
        mask_black = np.prod(mask_black, axis=-1, keepdims=True, dtype=np.uint8)
        mask_black = np.concatenate([mask_black, mask_black, mask_black], axis=-1)
        mask_black = cv.cvtColor(mask_black, cv.COLOR_BGR2GRAY)
        cv.imshow("maschera nera: ", mask_black)
        
        contours, _ = cv.findContours(mask_black, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        mask_black = cv.cvtColor(mask_black, cv.COLOR_GRAY2BGR)
        for cnt in contours:
            cv.polylines(cropped_img, [cnt], True, [0, 0, 255], 1)
            cv.polylines(mask_black, [cnt], True, [0, 0, 255], 1)

        objects_detected = []
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > self.area_threshold:
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
            #print("self.push_point (x, y)" + str(self.push_point))

            real_push_point = None
            if self.push_point:
                real_push_point = cv.perspectiveTransform(
                    np.array(self.push_point).reshape(1,1,2),
                    self.homography_matrix
                ).reshape(2).tolist()
                
                point_msg = Point()
                point_msg.x = real_push_point[0]
                point_msg.y = real_push_point[1]
                point_msg.z = 0.0
                # Pubblica la posizione
                self.push_point_pub.publish(point_msg)
            
            if self.start_end_points['start'] is None:
                self.start_end_points['start'] = self.push_point
                # homography and publish
                real_list2d = imgframe_to_worldframe(self.start_end_points['start'], self.homography_matrix)
                msg = list2d_to_point(real_list2d)
                self.initial_point_pub.publish(msg)

            elif (abs(prev_push_point[1]-self.push_point[1]) < self.pixel_tolerance 
                  and abs(self.start_end_points['start'][1]-self.push_point[1])> 5 
                  and self.counter_position <= self.threshold_wait):
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
                
                side_point = None #
                if self.counter_position == self.threshold_wait:
                    # Pubblica la distanza
                    self.distance_pub.publish(distance_norm)

                # print("Pixel distance: ", abs(distance))
                # print(f"Real distance: {real_distance}")
                # print(f"Distanza in norma:{distance_norm} ")
                #print(f"Matrice {self.homography_matrix}")

                draw_line_with_points(image=cropped_img, 
                              start_point=self.start_end_points['start'], 
                              end_point=self.start_end_points['end'],
                              side_point=None, # FIXME: remove
                              distance=f"{distance_norm:.2f}m")

            cv.circle(mask_black, (int(self.push_point[0]), int(self.push_point[1])), 1, (0, 255, 0), thickness=-1)
            cv.circle(cropped_img, (int(self.push_point[0]), int(self.push_point[1])), 1, (0, 255, 0), thickness=-1)
        
        # Show only the masked parts of the image
        #cv.imshow("Box frontal face", mask_black)
        cv.imshow("cropped", cropped_img)

        cv.waitKey(1)


## MAIN
if __name__ == '__main__':
    object_detection = ObjectDetection() 
    rospy.init_node('object_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()
