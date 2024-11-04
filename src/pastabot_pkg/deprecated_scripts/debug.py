import rospy
from std_msgs.msg import Bool, String 

rospy.init_node("debug", anonymous=True)
pub = rospy.Publisher("box/type_topic", String, queue_size=10)

# Attendi un breve tempo per permettere al publisher di inizializzarsi
rospy.sleep(0.5)

"""
No Object 
LIGHT BOX
MEDIUM BOX
HEAVY BOX
"""

# Ora pubblica il messaggio
pub.publish("LIGHT BOX")

# Aggiungi un log di conferma
rospy.loginfo("Messaggio pubblicato")
