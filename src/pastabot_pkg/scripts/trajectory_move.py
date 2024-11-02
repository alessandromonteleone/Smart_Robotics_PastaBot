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
CURRENT_ROBOT_POSE = [0.20, 0.0, 1.015]
START_POINT = [0.70, 0.0, 1.035]
STOP_POINT = [0.95, 0.0, 1.035]
HOME = [0.20, -0.40, 1.035]
last_clock_time = None 


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
    robot_arm.set_max_velocity_scaling_factor(1.0)
    robot_arm.set_max_acceleration_scaling_factor(1.0)
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
    vertical_quaternion = tf.quaternion_from_euler(math.radians(0.0), math.radians(180.0), math.radians(0.0))  # Roll, Pitch and Yaw


    # START PLAN to HOME POINT
    # HOME POINT
    home_pose = Pose()
    home_pose.position.x = HOME[0] - CURRENT_ROBOT_POSE[0]
    home_pose.position.y = HOME[1] - CURRENT_ROBOT_POSE[1]
    home_pose.position.z = HOME[2] - CURRENT_ROBOT_POSE[2]
    home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z, home_pose.orientation.w = vertical_quaternion

    # Setting and planning movements
    robot_arm.set_pose_target(home_pose)
    home_plan = robot_arm.plan()
    if home_plan:
        rospy.logwarn(f"Moving plan towards position ({home_pose.position.x:.2f}, {home_pose.position.y:.2f}, {home_pose.position.z:.2f}) generated successfully")
        if robot_arm.go(wait=True):
            rospy.logwarn("Movement done successfully")
            # continue
        else:
            rospy.logerr("Movement failed")
    else:
        rospy.logerr("Moving plan failed")
    robot_arm.stop()
    robot_arm.clear_pose_targets()


    # PUSH PLAN
    # Waypoints per la traiettoria continua
    waypoints = []
    waypoints.append(home_pose)
    
    # START POINT
    start_pose = Pose()
    start_pose.position.x = START_POINT[0] - CURRENT_ROBOT_POSE[0]
    start_pose.position.y = START_POINT[1] - CURRENT_ROBOT_POSE[1]
    start_pose.position.z = START_POINT[2] - CURRENT_ROBOT_POSE[2]
    start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w = vertical_quaternion
    waypoints.append(start_pose)
       
    # STOP POINT
    stop_pose = Pose()
    stop_pose.position.x = STOP_POINT[0] - CURRENT_ROBOT_POSE[0]
    stop_pose.position.y = STOP_POINT[1] - CURRENT_ROBOT_POSE[1]
    stop_pose.position.z = STOP_POINT[2] - CURRENT_ROBOT_POSE[2]
    stop_pose.orientation.x, stop_pose.orientation.y, stop_pose.orientation.z, stop_pose.orientation.w = vertical_quaternion
    waypoints.append(stop_pose)

    # Pianificazione della traiettoria cartesiana
    (push_plan, push_fraction) = robot_arm.compute_cartesian_path(waypoints, eef_step=0.01)


    # CYCLE
    while not rospy.is_shutdown():        
        #x, y, z = random_pose(base_x, base_y, base_z, min_z=0.00)

        # PUSH PLAN
        if push_fraction == 1.0:
            rospy.loginfo("Trajectory planned successfully, executing...")
            robot_arm.execute(push_plan, wait=True)
            rospy.loginfo("Trajectory execution completed successfully")
        else:
            rospy.logwarn(f"Trajectory planning was incomplete (fraction: {push_fraction*100:.2f}%)")
        robot_arm.stop()
        robot_arm.clear_pose_targets()

        
        # HOME PLAN
        # Setting and planning movements
        robot_arm.set_pose_target(home_pose)
        home_plan = robot_arm.plan()
        if home_plan:
            rospy.logwarn(f"Moving plan towards position ({home_pose.position.x:.2f}, {home_pose.position.y:.2f}, {home_pose.position.z:.2f}) generated successfully")
            if robot_arm.go(wait=True):
                rospy.logwarn("Movement done successfully")
                # continue
            else:
                rospy.logerr("Movement failed")
        else:
            rospy.logerr("Moving plan failed")
        robot_arm.stop()
        robot_arm.clear_pose_targets()

        
        rospy.sleep(15)


    # Shutdown MoveIt
    moveit_commander.roscpp_shutdown()


## MAIN
if __name__ == '__main__':
    try:
        # Calling function 
        robot_move()        
    except rospy.ROSInterruptException:
        pass
