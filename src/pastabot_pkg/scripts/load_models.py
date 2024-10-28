##!/usr/bin/env python3
import rospy
import rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import time
from object_detection import ObjectDetection
import cv2 as cv

class BoxSpawner:
    def __init__(self):
        rospy.init_node('box_spawner')
        
        # Attendi che i servizi di Gazebo siano disponibili
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.box_names = ['box_half_kg', 'box_one_kg', 'box_two_kg']
        
        # Ottieni il percorso del package
        self.rospack = rospkg.RosPack()
        print(os.getcwd())
        self.package_path = os.getcwd() + '/src/pastabot_pkg/'
        #self.package_path = self.rospack.get_path('pastabot_pkg')
        # Percorsi dei file SDF
        self.sdf_paths = {
            'light': os.path.join(self.package_path, 'models', 'box_half_kg', 'model.sdf'),
            'medium': os.path.join(self.package_path, 'models', 'box_one_kg', 'model.sdf'),
            'heavy': os.path.join(self.package_path, 'models', 'box_two_kg', 'model.sdf')
        }


    def delete_box(self, model_name="dynamic_box"):
        """Elimina il modello della scatola se esiste"""
        try:
            # Crea una richiesta di delete esplicita
            req = DeleteModelRequest()
            req.model_name = model_name
            
            # Esegui la delete con retry
            max_attempts = 5
            for attempt in range(max_attempts):
                try:
                    response = self.delete_model(req)
                    if response.success:
                        rospy.loginfo(f"Modello {model_name} eliminato con successo")
                        return True
                    else:
                        rospy.logwarn(f"Tentativo {attempt + 1}/{max_attempts}: Delete non riuscita per {model_name}")
                except rospy.ServiceException as e:
                    if attempt < max_attempts - 1:
                        rospy.logwarn(f"Tentativo {attempt + 1}/{max_attempts} fallito: {e}. Riprovo...")
                        time.sleep(1)  # Breve pausa prima del retry
                    else:
                        raise
            
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Errore fatale nella delete del modello {model_name}: {e}")
            return False

    def spawn_box(self, box_type='medium', position=(0.95, 0.0, 1.166), model_name="dynamic_box"):
        """
        Spawna una scatola del tipo specificato in una determinata posizione
        
        Args:
            box_type (str): 'light', 'medium', o 'heavy'
            position (tuple): (x, y, z) posizione della scatola
            model_name (str): nome del modello
        """
        if box_type not in self.sdf_paths:
            rospy.logwarn(f"Tipo di scatola non valido: {box_type}. Uso 'medium' come default.")
            box_type = 'medium'

        for model in self.box_names:
            # Prima elimina il modello esistente se presente
            self.delete_box(model)
            
        # Leggi il file SDF
        try:
            with open(self.sdf_paths[box_type], 'r') as f:
                sdf_string = f.read()
        except Exception as e:
            rospy.logerr(f"Errore nella lettura del file SDF: {e}")
            return
        
        # Definisci la posa del modello
        initial_pose = Pose()
        initial_pose.position = Point(*position)
        initial_pose.orientation = Quaternion(0, 0, 0, 1)
        
        try:
            # Spawna il modello in Gazebo
            response = self.spawn_model(
                model_name=model_name,
                model_xml=sdf_string,
                robot_namespace="/",
                initial_pose=initial_pose,
                reference_frame="world"
            )
            if response.success:
                rospy.loginfo(f"Scatola {box_type} spawnata con successo")
            else:
                rospy.logwarn(f"Errore nello spawn della scatola: {response.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Errore nello spawn del modello: {e}")

def main_old():
    try:
        spawner = BoxSpawner()
        # Attendi un secondo per assicurarti che tutto sia inizializzato
        rospy.sleep(1)  

        while (True):  # Continua fino a quando il nodo non viene chiuso
            try:
                choice = int(input("Insert 1 for 0.5kg box\nInsert 2 for 1kg box\nInsert 3 for 2kg box\nInsert 0 for Exit\nChoice:\n"))
            except ValueError: 
                print('Expected a number')
                continue  # Richiede nuovamente l'input
            
            if choice == 0:
                break
            print(list(spawner.sdf_paths.keys()))
            print(spawner.box_names)
            spawner.spawn_box(list(spawner.sdf_paths.keys())[choice-1], (0.95, 0.0, 1.166), spawner.box_names[choice-1])
            rospy.sleep(2)  # Attendi che il box venga spawnato
            
    except rospy.ROSInterruptException:
        pass


def main():
    try:
        spawner = BoxSpawner()
        # Attendi un secondo per assicurarti che tutto sia inizializzato
        rospy.sleep(1)  

        try:
            choice = int(input("Insert 1 for 0.5kg box\nInsert 2 for 1kg box\nInsert 3 for 2kg box\nInsert 0 for Exit\nChoice:\n"))
        except ValueError: 
            print('Expected a number')
            return
        
        spawner.spawn_box(list(spawner.sdf_paths.keys())[choice-1], (0.95, 0.0, 1.166), spawner.box_names[choice-1])
        rospy.sleep(2)

        ObjectDetection() 
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv.destroyAllWindows()
                
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()