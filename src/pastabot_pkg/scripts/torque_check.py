#!/usr/bin/env python3

import rospy
import math
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, WrenchStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber


# GLOBAL VARIABLES (Region boundaries in robot coordinates)
START_POINT = [0.61, -0.01, 0.01]
END_POINT = [0.70, 0.01, 0.03]
average_force = []
average_torque = []


## FUNCTIONS 
def synchronized_callback(ft_data, pose_data):
    # Extract force and torque values
    force = ft_data.wrench.force
    torque = ft_data.wrench.torque

    # Extract the current end-effector position
    position = pose_data.pose.position

    # Check if the end-effector is within the specified region
    if (START_POINT[0] <= position.x <= END_POINT[0] and
        START_POINT[1] <= position.y <= END_POINT[1]):
        
        # Calculate magnitudes and add them to lists for averaging
        force_magnitude = math.sqrt(force.x**2 + force.y**2 + force.z**2)
        torque_magnitude = math.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
        
        average_force.append(force_magnitude)
        average_torque.append(torque_magnitude)

    # If the end-effector moves out of the region, calculate and log averages
    elif average_force and average_torque:
        rospy.loginfo(f"{len(average_force)} measurements:")
        rospy.loginfo(f"--> Max Force: {max(average_force)}")
        rospy.loginfo(f"--> Max Torque: {max(average_torque)}")
        
        # Clear lists for the next region entry
        average_force.clear()
        average_torque.clear()

def print_force_and_torque_in_region():
    # Initialize ROS and MoveIt
    roscpp_initialize([])
    rospy.init_node("force_and_torque_printer", anonymous=True)
    
    # Initialize the MoveGroupCommander for controlling the robot arm
    robot_arm = MoveGroupCommander("gofa_group")
    
    # Create synchronized subscribers for force-torque data and end-effector pose
    ft_sub = Subscriber("/ft_sensor_topic", WrenchStamped)
    pose_sub = Subscriber("/gofa_group/get_current_pose", PoseStamped)
    
    # Synchronize messages with a tolerance of 0.1 seconds
    ats = ApproximateTimeSynchronizer([ft_sub, pose_sub], queue_size=10, slop=0.1)
    ats.registerCallback(synchronized_callback)

    # Keep node running until shutdown
    rospy.spin()
    roscpp_shutdown()

if __name__ == "__main__":
    print_force_and_torque_in_region()
