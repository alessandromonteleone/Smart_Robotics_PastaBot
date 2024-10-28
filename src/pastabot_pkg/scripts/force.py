from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3
import rospy

rospy.init_node('force_applier', anonymous=True)

def apply_force(model_name, link_name, force_vec, point_pos=(0,0,0), duration=1.0):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        
        wrench = Wrench()
        wrench.force = Vector3(force_vec[0], force_vec[1], force_vec[2])
        wrench.torque = Vector3(0, 0, 0)
        
        # Punto di applicazione della forza rispetto al centro di massa
        ref_point = Point(point_pos[0], point_pos[1], point_pos[2])
        
        start_time = rospy.Time().now()
        duration = rospy.Duration(duration)
        
        success = apply_wrench(
            body_name=f"{model_name}::{link_name}",
            reference_frame=f"{model_name}::{link_name}", # usa il frame del link
            reference_point=ref_point,
            wrench=wrench,
            start_time=start_time,
            duration=duration
        )
        
        print(f"Applied force {force_vec} to {model_name}::{link_name} at point {point_pos}")
        return success
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

# Uso:
apply_force("box_one_kg", "box_01_body", [1000.0, 0, 0], (0, 0, -0.100), 0.5)