#!/usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import time
from object_detection import ObjectDetection
import cv2 as cv


## GLOBAL VARIABLES
os.chdir(os.path.dirname(os.path.abspath(__file__)))


## CLASS
class BoxSpawner:
    def __init__(self):
        rospy.init_node('object_detector')
        
        # Wait for Gazebo services to be available
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.box_names = ['box_light', 'box_medium', 'box_heavy']
        
        # Get the package path
        self.rospack = rospkg.RosPack()
        self.package_path = '../'
        # Paths to SDF files
        self.sdf_paths = {
            'light': os.path.join(self.package_path, 'models', 'box_light', 'model.sdf'),
            'medium': os.path.join(self.package_path, 'models', 'box_medium', 'model.sdf'),
            'heavy': os.path.join(self.package_path, 'models', 'box_heavy', 'model.sdf')
        }

    def delete_box(self, model_name="dynamic_box"):
        # Delete the box model if it exists
        try:
            # Create an explicit delete request
            req = DeleteModelRequest()
            req.model_name = model_name
            
            # Perform delete with retry
            max_attempts = 2
            for attempt in range(max_attempts):
                try:
                    response = self.delete_model(req)
                    if response.success:
                        rospy.loginfo(f"Model {model_name} successfully deleted")
                        return True
                    else:
                        rospy.logwarn(f"Attempt {attempt + 1}/{max_attempts}: Delete failed for {model_name}")
                except rospy.ServiceException as e:
                    if attempt < max_attempts - 1:
                        rospy.logwarn(f"Attempt {attempt + 1}/{max_attempts} failed: {e}. Retrying...")
                        time.sleep(1)  # Short pause before retry
                    else:
                        raise
            
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Fatal error in deleting model {model_name}: {e}")
            return False

    def spawn_box(self, box_type='medium', position=(0.95, 0.0, 1.166), model_name="dynamic_box"):
        # Spawn a box of the specified type at a certain position
        """
        Args:
            box_type (str): 'light', 'medium', or 'heavy'
            position (tuple): (x, y, z) position of the box
            model_name (str): name of the model
        """
        if box_type not in self.sdf_paths:
            rospy.logwarn(f"Invalid box type: {box_type.upper()}. Using 'medium' as default.")
            box_type = 'medium'

        for model in self.box_names:
            # First delete the existing model if present
            self.delete_box(model)
        rospy.sleep(1)

        # Read the SDF file
        try:
            with open(self.sdf_paths[box_type], 'r') as f:
                sdf_string = f.read()
        except Exception as e:
            rospy.logerr(f"Error reading the SDF file: {e}")
            return
        
        # Define the model's pose
        initial_pose = Pose()
        initial_pose.position = Point(*position)
        initial_pose.orientation = Quaternion(0, 0, 0, 1)
        
        # Spawn the model in Gazebo
        try:
            response = self.spawn_model(
                model_name=model_name,
                model_xml=sdf_string,
                robot_namespace="/",
                initial_pose=initial_pose,
                reference_frame="world"
            )
            if response.success:
                rospy.loginfo(f"Spawned {box_type.upper()} BOX successfully")
            else:
                rospy.logwarn(f"Error while spawning the box: {response.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Error spawning model: {e}")


## FUNCTIONS
def load_models():
    try:
        spawner = BoxSpawner()
        # Wait a second to ensure everything is initialized
        rospy.sleep(1)  
        detector = ObjectDetection()

        while (True):  # Continue until the node is shut down
            try:
                choice = int(input("Insert 1 for light box\nInsert 2 for medium box\nInsert 3 for heavy box\nInsert 0 for Exit\nChoice:\n"))
            except ValueError: 
                print('Expected a number')
                continue  # Request input again
            
            if choice == 0:
                break
            #print(list(spawner.sdf_paths.keys()))
            #print(spawner.box_names)
            spawner.spawn_box(list(spawner.sdf_paths.keys())[choice-1], (0.95, 0.0, 1.166), spawner.box_names[choice-1])
            rospy.sleep(2)  # Wait for the box to be spawned
            detector.start()
            
    except rospy.ROSInterruptException as e:
        print(e)


'''
def main():
    try:
        spawner = BoxSpawner()
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
'''


## MAIN
if __name__ == '__main__':
    load_models()
