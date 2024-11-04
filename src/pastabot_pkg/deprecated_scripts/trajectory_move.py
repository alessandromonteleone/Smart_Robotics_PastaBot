#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf.transformations as tf
import math
from sensor_msgs.msg import JointState


## GLOBAL VARIABLES
START_POINT = [0.70, 0.0, 1.035]
STOP_POINT = [0.95, 0.0, 1.035]
HOME = [0.20, -0.40, 1.035]
last_clock_time = None 


## PARAMETERS
def reading_parameters():
    base_x = rospy.get_param("~arg_x", 0.0)  
    base_y = rospy.get_param("~arg_y", 0.0)
    base_z = rospy.get_param("~arg_z", 0.0)
    base_Roll = rospy.get_param("~arg_Roll", 0.0)
    base_Pitch = rospy.get_param("~arg_Pitch", 0.0)
    base_Yaw = rospy.get_param("~arg_Yaw", 0.0)
    rospy.loginfo(f"Current Robot Base 6D Pose: [{base_x}, {base_y}, {base_z}, {base_Roll}, {base_Pitch}, {base_Yaw}]")
    return [base_x, base_y, base_z, base_Roll, base_Pitch, base_Yaw]


## POINTS
def points(current_robot_pose):
    # Fixed orientation in quaternions
    vertical_quaternion = tf.quaternion_from_euler(math.radians(0.0), math.radians(180.0), math.radians(0.0))  # Roll, Pitch and Yaw

    # HOME POINT
    home_pose = Pose()
    home_pose.position.x = HOME[0] - current_robot_pose[0]
    home_pose.position.y = HOME[1] - current_robot_pose[1]
    home_pose.position.z = HOME[2] - current_robot_pose[2]
    home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z, home_pose.orientation.w = vertical_quaternion

    # START WEIGHT CHECK POINT
    start_weight_check_pose = Pose()
    start_weight_check_pose.position.x = START_POINT[0] - current_robot_pose[0]
    start_weight_check_pose.position.y = START_POINT[1] - current_robot_pose[1]
    start_weight_check_pose.position.z = START_POINT[2] - current_robot_pose[2]
    start_weight_check_pose.orientation.x, start_weight_check_pose.orientation.y, start_weight_check_pose.orientation.z, start_weight_check_pose.orientation.w = vertical_quaternion

    # STOP WEIGHT CHECK POINT
    stop_weight_check_pose = Pose()
    stop_weight_check_pose.position.x = STOP_POINT[0] - current_robot_pose[0]
    stop_weight_check_pose.position.y = STOP_POINT[1] - current_robot_pose[1]
    stop_weight_check_pose.position.z = STOP_POINT[2] - current_robot_pose[2]
    stop_weight_check_pose.orientation.x, stop_weight_check_pose.orientation.y, stop_weight_check_pose.orientation.z, stop_weight_check_pose.orientation.w = vertical_quaternion

    return home_pose, start_weight_check_pose, stop_weight_check_pose


## PLANS
def home_plan(robot, pose):
    robot.set_pose_target(pose)
    home_plan = robot.plan()
    if home_plan:
        rospy.logwarn(f"STARTING PLAN: moving plan towards position ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) generated successfully")
        if robot.go(wait=True):
            rospy.logwarn("STARTING PLAN: movement done successfully")
            # continue
        else:
            rospy.logerr("STARTING PLAN: movement failed")
    else:
        rospy.logerr("STARTING PLAN: moving plan failed")
    robot.stop()
    robot.clear_pose_targets()


def weight_check_plan(robot, waypoints=[]):
    plan, fraction = robot.compute_cartesian_path(waypoints, eef_step=0.01)
    if fraction == 1.0:
        rospy.loginfo("WEIGHT CHECK PLAN: trajectory planned successfully")
    else:
        rospy.logerr(f"WEIGHT CHECK PLAN: trajectory planning was incomplete (fraction: {fraction*100:.2f}%)")
    robot.stop()
    robot.clear_pose_targets()

    return plan


## MOVING ROBOT
def robot_move():
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
    home_pose, start_weight_check_pose, stop_weight_check_pose = points(current_robot_pose=reading_parameters())

    # HOME PLAN Definition and Execution
    home_plan(robot=robot_arm, pose=home_pose)    

    # WEIGHT CHECK PLAN Definition
    weight_push_plan = weight_check_plan(robot=robot_arm, waypoints=[home_pose, start_weight_check_pose, stop_weight_check_pose])

    # CYCLE
    while not rospy.is_shutdown():       

        # WEIGHT CHECK PLAN Execution
        if robot_arm.execute(weight_push_plan, wait=True):
            rospy.loginfo("WEIGHT CHECK PLAN: trajectory execution completed successfully")
        else:
            rospy.logerr("WEIGHT CHECK PLAN: trajectory execution failed")
        robot_arm.stop()
        robot_arm.clear_pose_targets()
        
        # HOME PLAN Definition and Execution
        home_plan(robot=robot_arm, pose=home_pose)     

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
