import rospy
from std_msgs.msg import Bool 

rospy.init_node("debug", anonymous=True)
pub = rospy.Publisher("force_check/stop_detector", Bool, queue_size=10)

# Attendi un breve tempo per permettere al publisher di inizializzarsi
rospy.sleep(0.5)

# Ora pubblica il messaggio
pub.publish(True)

# Aggiungi un log di conferma
rospy.loginfo("Messaggio pubblicato su 'force_check/stop_detector'")
