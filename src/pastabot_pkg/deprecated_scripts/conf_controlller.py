#!/usr/bin/env python3

import time
import rospy
from gazebo_msgs.srv import GetLinkProperties, SetLinkProperties, SetModelState, SetModelConfiguration
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Vector3
import math
from object_detection import ObjectDetection
import cv2 as cv
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

def set_model_mass(model_name, link_name, new_mass):
    rospy.wait_for_service('/gazebo/set_link_properties')

    try:
        get_link_props = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
        link_properties = get_link_props(model_name + '::' + link_name)

        set_link_props = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
        
        # Aggiusta i momenti di inerzia in base alla nuova massa
        mass_ratio = new_mass / link_properties.mass
        set_link_props(
            link_name=model_name + '::' + link_name,
            com=link_properties.com,
            gravity_mode=link_properties.gravity_mode,
            mass=new_mass,
            ixx=link_properties.ixx * mass_ratio,
            iyy=link_properties.iyy * mass_ratio,
            izz=link_properties.izz * mass_ratio,
            ixy=link_properties.ixy * mass_ratio,
            ixz=link_properties.ixz * mass_ratio,
            iyz=link_properties.iyz * mass_ratio
        )
        print(f"Set mass to {new_mass} for {model_name} ({link_name})")

        # Aggiungi un reset per aggiornare la fisica
        reset_physics = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_physics()

    except rospy.ServiceException as e:
        print(f"Error modifying link properties: {e}")

def set_model_state(model_name, position, roll, pitch, yaw):
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        model_state = ModelState()
        model_state.model_name = model_name
        
        # Set position
        model_state.pose = Pose()
        model_state.pose.position.x = position[0]
        model_state.pose.position.y = position[1]
        model_state.pose.position.z = position[2]

        # Convert roll, pitch, yaw to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        model_state.pose.orientation.w = cr * cp * cy + sr * sp * sy
        model_state.pose.orientation.x = sr * cp * cy - cr * sp * sy
        model_state.pose.orientation.y = cr * sp * cy + sr * cp * sy
        model_state.pose.orientation.z = cr * cp * sy - sr * sp * cy

        # Set velocity to zero
        model_state.twist = Twist()
        model_state.reference_frame = "world"

        # Call the service
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_state(model_state)
        print(f"Model {model_name} repositioned to {position} with orientation (roll, pitch, yaw): ({roll}, {pitch}, {yaw})")

    except rospy.ServiceException as e:
        print(f"Error modifying model state: {e}")



def process_inputs(input_dict):
    # Get parameters from dictionary
    model_name = input_dict['model_name']
    link_name = input_dict['link_name']
    new_mass = input_dict['mass']
    position = input_dict['position']
    roll = input_dict['roll']
    pitch = input_dict['pitch']
    yaw = input_dict['yaw']
    
    # Set the mass
    set_model_mass(model_name, link_name, new_mass)

    # Set model size
    #set_model_size(model_name, link_name, scale)

    # Set model position and orientation
    set_model_state(model_name, position, roll, pitch, yaw)


def callback(msg):
    pass


if __name__ == "__main__":


    rospy.init_node('main_node', anonymous=True)

    input_data = {
        'model_name': 'box_01_model',
        'link_name': 'box_01_body',
        'mass': 1,
        'position': [0.95, 0.0, 1.166],
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0
    }

    process_inputs(input_data)

    object_detection = ObjectDetection() 
    #rospy.init_node('object_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()
