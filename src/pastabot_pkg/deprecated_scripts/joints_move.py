#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

# Funzione per pubblicare forze sui giunti
def apply_joint_forces():
    # Inizializza il nodo ROS
    rospy.init_node('joint_force_controller', anonymous=True)
    
    # Crea un publisher per ciascun giunto
    joint_names = [
        'jnt1_rev', 'jnt2_rev', 'jnt3_rev', 
        'jnt4_rev', 'jnt5_rev', 'jnt6_rev'
    ]
    
    publishers = []
    
    for joint in joint_names:
        pub = rospy.Publisher(f'/{joint}_position_controller/command', Float64, queue_size=10)
        publishers.append(pub)

    # Attende che i publishers siano pronti
    rospy.sleep(1)

    # Applica forze ai giunti (valori di esempio)
    forces = [500.0, 500.0, 500.0, 500.0, 500.0, -500.0]  # Cambia i valori come desideri
    
    for i in range(len(joint_names)):
        rospy.loginfo(f"Applying force {forces[i]} to {joint_names[i]}")
        publishers[i].publish(forces[i])
        rospy.sleep(0.5)  # Pausa tra le applicazioni di forza

if __name__ == '__main__':
    try:
        apply_joint_forces()
    except rospy.ROSInterruptException:
        pass
