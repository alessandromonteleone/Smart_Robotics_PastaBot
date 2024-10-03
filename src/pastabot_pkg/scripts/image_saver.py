#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)                                              # ROS node initialization
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()                                                                    # CV Bridge for omages conversion between ROS anc CV images
        self.save_directory = "/home/luca/Scrivania/Smart-Robotics/Progetto/Smart_Robotics_PastaBot/src/pastabot_pkg/images"
        self.image_count = 0                                                                        # Images counter
        self.last_saved_time = time.time()                                                          # Time inizialization
        
    def image_callback(self, msg):
        current_time = time.time()  # Current time
        if current_time - self.last_saved_time >= 5:
            try:
                # Saving image
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                file_name = f"{self.save_directory}/image_{self.image_count}.png"
                cv2.imwrite(file_name, cv_image)
                rospy.loginfo(f"Saved image {file_name}")
                
                # Updating counter and time
                self.image_count += 1
                self.last_saved_time = current_time
            except Exception as e:
                rospy.logerr(f"Failed to save image: {e}")

    def run(self):
        # Executing node until the closing
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver = ImageSaver()
        image_saver.run()
    except rospy.ROSInterruptException:
        pass
