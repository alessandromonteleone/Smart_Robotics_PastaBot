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

def random_pose_within_reach(base_x, base_y, base_z, min_z=0.05):
    """Genera coordinate randomiche rispettando la distanza massima di 0.95 dalla base e z >= min_z."""
    while True:
        x = base_x + random.uniform(-0.95, 0.95)
        y = base_y + random.uniform(-0.95, 0.95)
        z = base_z + random.uniform(min_z, 0.95)  # z è sempre >= min_z

        if math.sqrt((x - base_x)**2 + (y - base_y)**2 + (z - base_z)**2) <= 0.95:
            return x, y, z

def add_table_collision(planning_scene):
    """Aggiunge un piano di collisione per rappresentare il tavolo su cui si trova il robot."""
    table_pose = PoseStamped()
    table_pose.header = Header(frame_id="world")
    table_pose.pose.position = Point(0.0, 0.0, 0.0)  # Posizione del piano a z = 0
    planning_scene.add_plane("table_plane", table_pose)

def move_gofa():
    # Inizializza ROS e MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_gofa_robot', anonymous=True)

    # Leggi la posizione della base del robot dai parametri
    base_x = rospy.get_param("~arg_x", 0.0)  
    base_y = rospy.get_param("~arg_y", 0.0)
    base_z = rospy.get_param("~arg_z", 0.0)

    print(base_x, " ", base_y, " ", base_z)

    # Inizializza il gruppo di pianificazione
    robot_arm = moveit_commander.MoveGroupCommander("gofa_group")
    planning_scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)  # Attende l'inizializzazione della scena

    # Aggiunge il piano di collisione per il tavolo
    add_table_collision(planning_scene)

    # Definisce l'orientamento fisso in quaternione (pitch di 180 gradi intorno a Y)
    roll, pitch, yaw = 0.0, math.radians(180.0), 0.0
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

    while not rospy.is_shutdown():
        # Genera una posizione randomica rispettando la distanza dalla base
        x, y, z = random_pose_within_reach(base_x, base_y, base_z, min_z=0.05)

        # Definisce la destinazione di posa
        pose_target = Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]

        # Imposta e pianifica il movimento
        robot_arm.set_pose_target(pose_target)
        plan = robot_arm.plan()

        if plan:
            rospy.loginfo(f"Piano di movimento verso posizione ({x:.2f}, {y:.2f}, {z:.2f}) generato con successo.")
            if robot_arm.go(wait=True):
                rospy.loginfo("Movimento eseguito con successo.")
                continue  # Genera subito una nuova posizione
            else:
                rospy.logwarn("Il movimento non è riuscito.")
        else:
            rospy.logwarn("Piano di movimento non riuscito.")

        # Ferma il braccio e cancella il target
        robot_arm.stop()
        robot_arm.clear_pose_targets()

    # Shutdown di MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_gofa()
    except rospy.ROSInterruptException:
        pass
