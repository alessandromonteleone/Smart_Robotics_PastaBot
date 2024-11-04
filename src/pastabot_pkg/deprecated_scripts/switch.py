#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import SwitchController

def switch_to_position_control():
    rospy.wait_for_service('/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        switch_controller(start_controllers=['position'],
                          stop_controllers=['effort'],
                          strictness=1)
        rospy.loginfo("Switched to position control.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('controller_switch_node')
    
    # Attendere un tempo specificato prima di passare al controllo di posizione
    rospy.sleep(5)  # 5 secondi di controllo di sforzo
    
    switch_to_position_control()

if __name__ == '__main__':
    main()
