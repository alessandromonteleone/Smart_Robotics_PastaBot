#!/usr/bin/env python3

import rospy
import math
import time
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, WrenchStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber


## GLOBAL VARIABLES (Punti rispetto alle coordinate del robot) 
CURRENT_ROBOT_POSE = [0.20, 0.0, 1.015]
START_POINT = [0.87, -0.01, 1.025]
END_POINT = [0.93, 0.01, 1.045]

# Variabili globali per memorizzare i valori di forza e coppia
current_force = None
current_torque = None


## CALLBACK FUNCTION
def ft_sensor_callback(data):
    global current_force, current_torque
    # Estrai forza e coppia dal messaggio
    current_force = data.wrench.force
    current_torque = data.wrench.torque


## FUNCTIONS
def print_force_and_torque_in_region():
    # Inizializza ROS e MoveIt
    roscpp_initialize([])
    rospy.init_node("force_and_torque_printer", anonymous=True)
    
    # Configura il gruppo di movimento del braccio
    robot_arm = MoveGroupCommander("gofa_group") 
    
    # Subscriber per il topic /ft_sensor_topic
    rospy.Subscriber("/ft_sensor_topic", WrenchStamped, ft_sensor_callback)
    
    average_force = []
    average_torque = []    
    calculation_done = True
    try:
        while not rospy.is_shutdown():
            # Ottieni la posizione attuale dell'end-effector
            current_pose: PoseStamped = robot_arm.get_current_pose()
            position = current_pose.pose.position
            
            # Verifica se l'end-effector è all'interno della regione di interesse
            if ((START_POINT[0] - CURRENT_ROBOT_POSE[0]) <= position.x <= (END_POINT[0] - CURRENT_ROBOT_POSE[0]) and
                (START_POINT[1] - CURRENT_ROBOT_POSE[1]) <= position.y <= (END_POINT[1] - CURRENT_ROBOT_POSE[1]) and
                (START_POINT[2] - CURRENT_ROBOT_POSE[2]) <= position.z <= (END_POINT[2] - CURRENT_ROBOT_POSE[2])):
                calculation_done = False
                rospy.loginfo("Checking force and torque")

                
                # Stampa la forza e la coppia solo se sono disponibili
                if current_force and current_torque:
                    average_force.append(math.sqrt((current_force.x)**2))
                    average_torque.append(math.sqrt((current_torque.x)**2))

            elif not calculation_done:
                rospy.loginfo(f"{len(average_force)} measurements:")

                # Calcola la media dei 5 valori maggiori per forza e coppia
                if len(average_force) >= 5:
                    top_5_forces = sorted(average_force, reverse=True)[:5]
                    average_top_5_force = sum(top_5_forces) / len(top_5_forces)
                    rospy.loginfo(f"--> Average of Top 5 Forces: {average_top_5_force}")
                else:
                    rospy.loginfo("Not enough force measurements for Top 5 average")

                if len(average_torque) >= 5:
                    top_5_torques = sorted(average_torque, reverse=True)[:5]
                    average_top_5_torque = sum(top_5_torques) / len(top_5_torques)
                    rospy.loginfo(f"--> Average of Top 5 Torques: {average_top_5_torque}")
                else:
                    rospy.loginfo("Not enough torque measurements for Top 5 average")

                # Reset lists after calculations
                average_force.clear()
                average_torque.clear()
                calculation_done = True
            
            # Aspetta un breve intervallo per evitare un flusso continuo
            time.sleep(0.001)  # Cambia l'intervallo per aggiornamenti più o meno frequenti
            
    except rospy.ROSInterruptException:
        pass
    finally:
        # Arresta MoveIt in modo sicuro
        roscpp_shutdown()

if __name__ == "__main__":
    print_force_and_torque_in_region()
