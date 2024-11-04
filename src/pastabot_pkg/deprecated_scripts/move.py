#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import tf.transformations as tf
import math

def move_gofa():
    # Inizializza il nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_gofa_robot', anonymous=True)

    # Inizializza il gruppo di pianificazione
    robot_arm = moveit_commander.MoveGroupCommander("gofa_group")

    # Visualizza lo stato attuale del robot
    rospy.loginfo("Stato corrente del robot:")
    rospy.loginfo(robot_arm.get_current_state())

    # Definisci un target di posizione per il braccio
    pose_target = Pose()
    pose_target.position.x = 0.5
    pose_target.position.y = -0.6
    pose_target.position.z = 0.0
    # NOTA: adesso dovrò definire tutti i possibili punti presenti all'interno di un raggio di 0.95 (pitagora),
    # con z negativo e con lo stelo del gripper posto sempre e solo verticalmente come qui.

    # Definisci l'orientamento in angoli di Eulero (in gradi, poi convertiti in radianti)
    roll = math.radians(0.0)    # Rotazione intorno all'asse X
    pitch = math.radians(180.0)  # Rotazione intorno all'asse Y
    yaw = math.radians(0-0)    # Rotazione intorno all'asse Z

    # Converte gli angoli di Eulero in quaternioni
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

    # Imposta l'orientamento nel formato quaternion
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]

    # Imposta la posizione di destinazione
    robot_arm.set_pose_target(pose_target)

    # Pianifica il movimento
    plan = robot_arm.plan()

    # Controlla se il piano è stato generato con successo
    if plan:
        rospy.loginfo("Piano di movimento pianificato con successo.")
        
        # Esegui il movimento
        success = robot_arm.go(wait=True)
        
        if success:
            rospy.loginfo("Movimento eseguito con successo.")
        else:
            rospy.logwarn("Il movimento non è riuscito.")
    else:
        rospy.logwarn("Piano di movimento non riuscito.")

    # Ferma il braccio dopo il movimento
    robot_arm.stop()

    # Cancella il target di posizione
    robot_arm.clear_pose_targets()

    rospy.loginfo("Movimento completato.")

    # Shutdown di MoveIt
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        move_gofa()
    except rospy.ROSInterruptException:
        pass
