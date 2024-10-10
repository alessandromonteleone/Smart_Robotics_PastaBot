#!/usr/bin/python3
import rospkg
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os 

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)  # Inizializzazione nodo ROS
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()  # CV Bridge per la conversione delle immagini tra ROS e OpenCV
        rospack = rospkg.RosPack()
        self.save_directory = os.path.join(rospack.get_path('pastabot_pkg'), 'images')
        
        # Controlla che la directory esista, altrimenti creala
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        
        print(f"Salvataggio immagini nella directory: {self.save_directory}")

        self.image_count = 0  # Contatore delle immagini
        self.last_saved_time = time.time()  # Tempo dell'ultima immagi    
            
    def image_callback(self, msg):
        current_time = time.time()  # Current time
        if current_time - self.last_saved_time >= 10:
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

    
