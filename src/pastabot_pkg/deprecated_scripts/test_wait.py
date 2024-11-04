import rospy
from position_manager import PositionManager
import time
from std_msgs.msg import String 


rospy.init_node('test_wait', anonymous=True)
pub = rospy.Publisher("box/type_topic", String, queue_size=10)
position_manager = PositionManager()         

while not rospy.is_shutdown():
    push_point = position_manager.wait_for_initial_push_point()

    time.sleep(10) # simulating robot movement
    pub.publish("HEAVY BOX")
    rospy.loginfo("Messaggio 'HEAVY BOX' pubblicato")

    side_point, dest = position_manager.wait_for_side_point_and_dest()
    rospy.loginfo(f"Response: {side_point=}, {dest=}")

    time.sleep(10)
    pub.publish("No Object")

    rospy.loginfo("Messaggio 'No Object' pubblicato")
    position_manager.reset()