#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Point
import tf.transformations as tf
import math
import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


## FUNCTIONS
def add_collision_plan(planning_scene):
    # Adding a Collision Plan to avoid any contact of the robot with the underlying table
    table_pose = PoseStamped()
    table_pose.header = Header(frame_id="world")
    table_pose.pose.position = Point(0.0, 0.0, 1.015)
    planning_scene.add_plane("table_plane", table_pose)


def add_joint_constraints(robot_arm, min_z):
    # Adds Joint Constraints to prevent joints from going below the table level
    constraints = moveit_msgs.msg.Constraints()
    constraints.name = "keep_joints_above_table"
    joint_names = robot_arm.get_joints()
    
    rospy.loginfo("Applying constraints to joints to keep them above the table level.")
    for joint_name in joint_names:

        
        if 'rev' in joint_name:  # Apply the constraint only to relevant joints
            joint_constraint = moveit_msgs.msg.JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = min_z  # Set minimum height
            joint_constraint.tolerance_below = 0.0  # No tolerance below the table level
            constraints.joint_constraints.append(joint_constraint)
    
    # Apply constraints to the robot arm
    robot_arm.set_path_constraints(constraints)


def random_pose(base_x, base_y, base_z, min_z=0.05):
    # Random Coordinates within maximum distance of 0.95 meters
    min_radius = 0.40
    max_radius = 0.95
    while True:
        x = base_x + random.uniform(-max_radius, max_radius)
        y = base_y + random.uniform(-max_radius, max_radius)
        z = base_z + random.uniform(min_z, 0.2)  # z è sempre >= min_z

        if min_radius <= math.sqrt((x - base_x)**2 + (y - base_y)**2 + (z - base_z)**2) <= max_radius:
            return x, y, z


def robot_move():
    # ROS, MoveIt and PlanningSceneInterface initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_gofa_robot', anonymous=True)
    rospy.loginfo("Waiting for MoveIta and necessary topics activation")
    rospy.wait_for_message('/joint_states', JointState)
    robot_arm = moveit_commander.MoveGroupCommander("gofa_group")
    # robot_arm.set_planner_id("RRTConnect")
    # robot_arm.set_planning_time(15.0)
    # robot_arm.set_goal_tolerance(0.01)
    # robot_arm.allow_replanning(True)
    # robot_arm.set_num_planning_attempts(10)
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
    
    # Adding collision plan for table
    add_collision_plan(planning_scene)

     # Adding joint constraints to avoid going below the table level
    add_joint_constraints(robot_arm, 0.0)

    # Fixed orientation in Quaternions
    roll, pitch, yaw = 0.0, math.radians(180.0), 0.0
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

    # Generating random position
    while not rospy.is_shutdown():
        x, y, z = random_pose(base_x, base_y, base_z, min_z=0.05)

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
