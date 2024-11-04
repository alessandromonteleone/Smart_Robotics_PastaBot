#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf.transformations as tf
import math
import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


## GLOBAL VARIABLES
last_clock_time = None  # Variabile per tenere traccia dell'ultimo tempo ricevuto


## FUNCTIONS
def random_pose(base_x, base_y, base_z, min_z=0.00):
    # Random Coordinates within maximum distance of 0.95 meters
    min_radius = 0.40
    max_radius = 0.95
    while True:
        x = base_x + random.uniform(0.35, max_radius)
        y = base_y + random.uniform(-0.40, 0.40)
        z = base_z + random.uniform(min_z, 0.1)  # z è sempre >= min_z

        if min_radius <= math.sqrt((x - base_x)**2 + (y - base_y)**2 + (z - base_z)**2) <= max_radius:
            return x, y, z


def clock_callback(data):
    global last_clock_time
    last_clock_time = data.clock.to_sec()


def robot_move():
    # Subscrbing to /clock topic of Gazebo 
    rospy.Subscriber('/clock', Clock, clock_callback)  ###TODO

    # ROS, MoveIt and PlanningSceneInterface initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_gofa_robot', anonymous=True)
    rospy.loginfo("Waiting for MoveIt and necessary topics activation")
    rospy.wait_for_message('/joint_states', JointState)
    robot_arm = moveit_commander.MoveGroupCommander("gofa_group")
    robot_arm.set_planning_time(1.0)
    planning_scene = moveit_commander.PlanningSceneInterface()
    rospy.loginfo(f"Waiting for service {planning_scene}...")
    rospy.sleep(1)
    rospy.loginfo("Continuous Move Initialization completed")

    # Reading Robot Base 6D Pose
    # base_x = rospy.get_param("~arg_x", 0.0)  
    # base_y = rospy.get_param("~arg_y", 0.0)
    # base_z = rospy.get_param("~arg_z", 0.0)
    # base_Roll = rospy.get_param("~arg_Roll", 0.0)
    # base_Pitch = rospy.get_param("~arg_Pitch", 0.0)
    # base_Yaw = rospy.get_param("~arg_Yaw", 0.0)
    # rospy.loginfo(f"Current Robot Base 6D Pose: [{base_x}, {base_y}, {base_z}, {base_Roll}, {base_Pitch}, {base_Yaw}]")
    base_x = base_y = base_z = 0.0

    # Fixed orientation in Quaternions
    roll, pitch, yaw = 0.0, math.radians(180.0), 0.0
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

    # Generating random position
    while not rospy.is_shutdown():
        x, y, z = random_pose(base_x, base_y, base_z, min_z=0.00)

        # Pose destination
        pose_target = Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]

        # Setting and planning movements
        robot_arm.set_pose_target(pose_target)
        plan = robot_arm.plan()

        if plan:
            rospy.logwarn(f"Moving plan towards position ({x:.2f}, {y:.2f}, {z:.2f}) generated successfully")
            if robot_arm.go(wait=True):
                rospy.logwarn("Movement done successfully")
                continue
            else:
                rospy.logerr("Movement failed")
        else:
            rospy.logerr("Moving plan failed")
            
        # Stopping robot arm and deleting target
        robot_arm.stop()
        robot_arm.clear_pose_targets()

    # Shutdown MoveIt
    moveit_commander.roscpp_shutdown()


## MAIN
if __name__ == '__main__':
    try:
        # Calling function 
        robot_move()        
    except rospy.ROSInterruptException:
        pass
