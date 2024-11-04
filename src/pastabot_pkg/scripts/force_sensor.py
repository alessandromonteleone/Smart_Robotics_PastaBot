#!/usr/bin/env python3

import math
import rospy
import time
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, WrenchStamped
from position_manager import PositionManager
from std_msgs.msg import String


## GLOBAL VARIABLES
START_POINT_OFFSET = [0.0, -0.01, 1.025]
END_POINT_OFFEST = [0.02, 0.01, 1.045]    
THRESHOLD_NO_OBJECT = 0.5
THRESHOLD_EMPTY_LIGHT = 0.2
THRESHOLD_LIGHT_MEDIUM = 2.0
THRESHOLD_MEDIUM_MAX = 4.25
current_force = None
current_torque = None


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


## CALLBACK FUNCTION
def ft_sensor_callback(data):
    global current_force, current_torque
    current_force = data.wrench.force
    current_torque = data.wrench.torque


## PUSH REGION DEFINITION
def push_check_region(point):
    start_point = [round(point.x, 3) + START_POINT_OFFSET[0], 
                   round(point.y, 3) + START_POINT_OFFSET[1], 
                   round(point.z, 3) + START_POINT_OFFSET[2]]
    end_point = [round(point.x, 3) + END_POINT_OFFEST[0], 
                  round(point.y, 3) + END_POINT_OFFEST[1], 
                  round(point.z, 3) + END_POINT_OFFEST[2]]
    
    return start_point, end_point


## FUNCTIONS
def print_force_and_torque_in_region():
    # ROS, MoveIt and PubSub initialization
    roscpp_initialize([])
    rospy.init_node("force_sensor", anonymous=True)
    rospy.loginfo("Waiting for MoveIt and necessary topics activation")
    rospy.wait_for_message('/joint_states', JointState)
    robot_arm = MoveGroupCommander("gofa_group")
    box_type_publisher = rospy.Publisher("/box/type_topic", String, queue_size=10)
    box_type_and_inertia_publisher = rospy.Publisher("/box/type_and_inertia_topic", String, queue_size=10)
    rospy.Subscriber("/ft_sensor_topic", WrenchStamped, ft_sensor_callback)
    position_manager = PositionManager()                            # Instancing Postion Manager
            
    # Waiting for WEIGHT CHECK PUSH POINT
    push_point = position_manager.wait_for_initial_push_point()
    start_push_check_region_point, end_push_check_region_point = push_check_region(point=push_point)
    current_robot_pose = reading_parameters()[:3]

    # CYCLE
    average_force = []
    average_torque = []    
    calculation_done = True
    while not rospy.is_shutdown():

        # Current END EFFECTOR Pose
        current_pose: PoseStamped = robot_arm.get_current_pose()
        position = current_pose.pose.position
        
        # Checking END EFFECTOR Pose inside Weight Check Region
        if ((start_push_check_region_point[0] - current_robot_pose[0]) <= position.x <= (end_push_check_region_point[0] - current_robot_pose[0]) and
            (start_push_check_region_point[1] - current_robot_pose[1]) <= position.y <= (end_push_check_region_point[1] - current_robot_pose[1]) and
            (start_push_check_region_point[2] - current_robot_pose[2]) <= position.z <= (end_push_check_region_point[2] - current_robot_pose[2])):
            calculation_done = False

            if current_force and current_torque:  # If available
                average_force.append(math.sqrt((current_force.x)**2))
                average_torque.append(math.sqrt((current_torque.x)**2))

        # First iteration after exiting Pushing Region
        elif not calculation_done:
            
            # Average of 5 HIGHEST VALUES and BOX CATEGORY
            if len(average_force) >= 5:
                top_5_forces = sorted(average_force, reverse=True)[:5]
                average_top_5_force = sum(top_5_forces) / len(top_5_forces)
                rospy.loginfo(f"--> Average of Top 5 Forces: {average_top_5_force}")
                if average_top_5_force < THRESHOLD_EMPTY_LIGHT:
                    box_type = "Empty Table"
                elif THRESHOLD_EMPTY_LIGHT <= average_top_5_force < THRESHOLD_LIGHT_MEDIUM:
                    box_type = "LIGHT BOX"
                    box_type_publisher.publish(box_type)
                    rospy.loginfo(f"Published box type: {box_type}")
                elif THRESHOLD_LIGHT_MEDIUM <= average_top_5_force < THRESHOLD_MEDIUM_MAX:
                    box_type = "MEDIUM BOX"
                    box_type_publisher.publish(box_type)
                    rospy.loginfo(f"Published box type: {box_type}")
                else:
                    box_type = "HEAVY BOX"
                    box_type_publisher.publish(box_type)
                    rospy.loginfo(f"Published box type: {box_type}")
                box_type_and_inertia_publisher.publish(f"Box type: {box_type}; Inertia: {round(average_top_5_force, 3)} N")

            else:
                rospy.loginfo("Not enough force measurements for Top 5 average")

            # Reset lists after calculations
            average_force.clear()
            average_torque.clear()
            calculation_done = True

        
        # Check interval
        time.sleep(0.001)

    # Shutdown
    roscpp_shutdown()


## MAIN
if __name__ == "__main__":
    try:
        # Calling function 
        print_force_and_torque_in_region()   
    except rospy.ROSInterruptException:
        pass
