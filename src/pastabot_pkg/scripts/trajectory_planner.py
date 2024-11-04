#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf.transformations as tf
import math
from sensor_msgs.msg import JointState
from position_manager import PositionManager


## GLOBAL VARIABLES
HOME_POINT = [0.20, -0.40, 1.035]
START_POINT_OFFSET = [-0.21, 0.0, 1.035]
STOP_POINT_OFFEST = [0.04, 0.0, 1.035]


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
def point(current_point, current_robot_pose):
    # Fixed orientation in quaternions
    vertical_quaternion = tf.quaternion_from_euler(math.radians(0.0), math.radians(180.0), math.radians(0.0))  # Roll, Pitch and Yaw
    point_pose = Pose()
    point_pose.position.x = current_point[0] - current_robot_pose[0]
    point_pose.position.y = current_point[1] - current_robot_pose[1]
    point_pose.position.z = current_point[2] - current_robot_pose[2]
    point_pose.orientation.x, point_pose.orientation.y, point_pose.orientation.z, point_pose.orientation.w = vertical_quaternion

    return point_pose


## PLANS
def home_plan(robot, pose):
    robot.set_pose_target(pose)
    home_plan = robot.plan()
    if home_plan:
        rospy.logwarn(f"HOME PLAN: moving plan towards position ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) generated successfully")
        if robot.go(wait=True):
            rospy.logwarn("HOME PLAN: movement done successfully")
            # continue
        else:
            rospy.logerr("HOME PLAN: movement failed")
    else:
        rospy.logerr("HOME PLAN: moving plan failed")
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
    rospy.init_node('trajectory_planner', anonymous=True)
    rospy.loginfo("Waiting for MoveIt and necessary topics activation")
    rospy.wait_for_message('/joint_states', JointState)
    robot_arm = moveit_commander.MoveGroupCommander("gofa_group")
    robot_arm.set_planning_time(1.0)
    robot_arm.set_max_velocity_scaling_factor(1.0)
    robot_arm.set_max_acceleration_scaling_factor(1.0)
    planning_scene = moveit_commander.PlanningSceneInterface()
    position_manager = PositionManager()                            # Instancing Postion Manager
    rospy.loginfo(f"Waiting for service {planning_scene}...")
    rospy.sleep(1)
    rospy.loginfo("Continuous Move Initialization completed")

    # Reading Robot Base 6D Pose
    home_pose = point(current_point=HOME_POINT, current_robot_pose=reading_parameters())

    # HOME PLAN Definition & Execution
    home_plan(robot=robot_arm, pose=home_pose)    

    # CYCLE
    while not rospy.is_shutdown():       

        # Waiting for WEIGHT CHECK PUSH POINT
        push_point = position_manager.wait_for_initial_push_point()
        start_weight_check_pose = point(
            current_point=[round(push_point.x, 3) + START_POINT_OFFSET[0], 
                           round(push_point.y, 3) + START_POINT_OFFSET[1], 
                           round(push_point.z, 3) + START_POINT_OFFSET[2]], 
            current_robot_pose=reading_parameters())
        stop_weight_check_pose = point(
            current_point=[round(push_point.x, 3) + STOP_POINT_OFFEST[0], 
                           round(push_point.y, 3) + STOP_POINT_OFFEST[1], 
                           round(push_point.z, 3) + STOP_POINT_OFFEST[2]],
            current_robot_pose=reading_parameters())
        rospy.loginfo(f"START PUSH POINT: {stop_weight_check_pose.position}")
        rospy.loginfo(f"STOP PUSH POINT: {stop_weight_check_pose.position}")

        # WEIGHT CHECK PLAN Definition & Execution
        weight_push_plan = weight_check_plan(robot=robot_arm, waypoints=[home_pose, start_weight_check_pose, stop_weight_check_pose])
        if robot_arm.execute(weight_push_plan, wait=True):
            rospy.loginfo("WEIGHT CHECK PLAN: trajectory execution completed successfully")
        else:
            rospy.logerr("WEIGHT CHECK PLAN: trajectory execution failed")
        robot_arm.stop()
        robot_arm.clear_pose_targets()
        
        # HOME PLAN Definition & Execution
        home_plan(robot=robot_arm, pose=home_pose)

        # Resetting Position Manager at the end of each iteration
        position_manager.reset()        

    # Shutdown
    moveit_commander.roscpp_shutdown()


## MAIN
if __name__ == '__main__':
    try:
        # Calling function 
        robot_move()        
    except rospy.ROSInterruptException:
        pass