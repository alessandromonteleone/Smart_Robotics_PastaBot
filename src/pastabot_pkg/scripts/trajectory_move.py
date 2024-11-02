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
SWITCH_POINT = [0.45, 0.0, 1.015]
INTERMEDIATE_POINT = [0.70, 0.0, 1.035]
STOP_POINT = [0.90, 0.0, 1.035]
HOME = [0.20, 0.40, 1.035]
last_clock_time = None  # Variabile per tenere traccia dell'ultimo tempo ricevuto


## FUNCTIONS
def random_pose(base_x, base_y, base_z, min_z=0.00):
    # Random Coordinates within maximum distance of 0.95 meters
    min_radius = 0.40
    max_radius = 0.95
    while True:
        x = base_x + random.uniform(0.85, max_radius)
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
    base_x = rospy.get_param("~arg_x", 0.0)  
    base_y = rospy.get_param("~arg_y", 0.0)
    base_z = rospy.get_param("~arg_z", 0.0)
    base_Roll = rospy.get_param("~arg_Roll", 0.0)
    base_Pitch = rospy.get_param("~arg_Pitch", 0.0)
    base_Yaw = rospy.get_param("~arg_Yaw", 0.0)
    rospy.loginfo(f"Current Robot Base 6D Pose: [{base_x}, {base_y}, {base_z}, {base_Roll}, {base_Pitch}, {base_Yaw}]")

    # Fixed orientation in Quaternions
    roll, pitch, yaw = 0.0, math.radians(180.0), 0.0
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

    # Generating random position
    while not rospy.is_shutdown():        
        #x, y, z = random_pose(base_x, base_y, base_z, min_z=0.00)


        rospy.sleep(20)


        # Waypoints per la traiettoria continua
        waypoints = []
    
        # SWITCH POINT
        switch_pose = Pose()
        switch_pose.position.x = SWITCH_POINT[0] - 0.20
        switch_pose.position.y = SWITCH_POINT[1] - 0.0
        switch_pose.position.z = SWITCH_POINT[2] - 1.015
        switch_pose.orientation.x, switch_pose.orientation.y, switch_pose.orientation.z, switch_pose.orientation.w = quaternion
        waypoints.append(switch_pose)
        
        # INTERMEDIATE POINT
        intermediate_pose = Pose()
        intermediate_pose.position.x = INTERMEDIATE_POINT[0] - 0.20
        intermediate_pose.position.y = INTERMEDIATE_POINT[1] - 0.0
        intermediate_pose.position.z = INTERMEDIATE_POINT[2] - 1.015
        intermediate_pose.orientation.x, intermediate_pose.orientation.y, intermediate_pose.orientation.z, intermediate_pose.orientation.w = quaternion
        waypoints.append(intermediate_pose)
        
        # PUSH POINT
        stop_pose = Pose()
        stop_pose.position.x = STOP_POINT[0] - 0.20
        stop_pose.position.y = STOP_POINT[1] - 0.0
        stop_pose.position.z = STOP_POINT[2] - 1.015
        stop_pose.orientation.x, stop_pose.orientation.y, stop_pose.orientation.z, stop_pose.orientation.w = quaternion
        waypoints.append(stop_pose)

        # Pianificazione della traiettoria cartesiana
        (plan, fraction) = robot_arm.compute_cartesian_path(waypoints, eef_step=0.01)

        if fraction == 1.0:
            rospy.loginfo("Trajectory planned successfully, executing...")
            robot_arm.execute(plan, wait=True)
            rospy.loginfo("Trajectory execution completed successfully")
        else:
            rospy.logwarn(f"Trajectory planning was incomplete (fraction: {fraction*100:.2f}%)")

        # Stopping robot arm and deleting target
        robot_arm.stop()
        robot_arm.clear_pose_targets()

        ########################################################################################### HOME
        pose_target = Pose()
        pose_target.position.x = HOME[0] - 0.20
        pose_target.position.y = HOME[1] - 0.0
        pose_target.position.z = HOME[2] - 1.015
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]

        # Setting and planning movements
        robot_arm.set_pose_target(pose_target)
        plan = robot_arm.plan()

        if plan:
            rospy.logwarn(f"Moving plan towards position ({pose_target.position.x:.2f}, {pose_target.position.y:.2f}, {pose_target.position.z:.2f}) generated successfully")
            if robot_arm.go(wait=True):
                rospy.logwarn("Movement done successfully")
                # continue
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
