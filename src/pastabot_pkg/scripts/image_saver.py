#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSaver:
    def __init__(self):
        # Inizializza il nodo ROS
        rospy.init_node('image_saver', anonymous=True)
        
        # Sottoscriviti al topic della camera
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Istanzia cv_bridge per convertire tra immagini ROS e OpenCV
        self.bridge = CvBridge()
        
        # Tempo dell'ultima immagine salvata
        self.last_saved_time = time.time()
        
        # Directory in cui salvare le immagini
        self.save_directory = "/home/luca/Scrivania/Smart-Robotics/Progetto/Smart_Robotics_PastaBot/src/pastabot_pkg/images"
        
        # Contatore per numerare le immagini
        self.image_count = 0
        
    def image_callback(self, msg):
        # Ottieni l'ora corrente
        current_time = time.time()

        # Salva un'immagine solo se sono passati almeno 5 secondi
        if current_time - self.last_saved_time >= 5:
            try:
                # Converti il messaggio ROS in un'immagine OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # Crea il nome del file
                file_name = f"{self.save_directory}/image_{self.image_count}.png"
                
                # Salva l'immagine
                cv2.imwrite(file_name, cv_image)
                
                # Stampa un messaggio di conferma
                rospy.loginfo(f"Saved image {file_name}")
                
                # Aggiorna il contatore e il tempo
                self.image_count += 1
                self.last_saved_time = current_time

            except Exception as e:
                rospy.logerr(f"Failed to save image: {e}")

    def run(self):
        # Mantieni il nodo in esecuzione fino alla chiusura
        rospy.spin()

if __name__ == '__main__':
    try:
        # Crea un oggetto ImageSaver e avvialo
        image_saver = ImageSaver()
        image_saver.run()
    except rospy.ROSInterruptException:
        pass
