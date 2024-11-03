#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from time import sleep


## CLASS
class PositionManager:
    def __init__(self):
        self.initial_point_sub = rospy.Subscriber("box/initial_point", Point, self.initial_point_callback)
        self.push_point_sub = rospy.Subscriber("box/push_point", Point, self.push_point_callback)
        self.side_point_sub = rospy.Subscriber("box/side_point", Point, self.side_point_callback)
        self.dest_sub = rospy.Subscriber("box/dest", String, self.dest_callback)

        self.reset()

    def reset(self):
        self.initial_push_point = None
        self.current_push_point = None
        self.side_point = None
        self.dest = None

    def push_point_callback(self, msg):
        self.current_push_point = msg

    def initial_point_callback(self, msg):
        self.initial_push_point = msg

    def side_point_callback(self, msg):
        self.side_point = msg
        rospy.loginfo(f"Side point saved: x={msg.x}, y={msg.y}, z={msg.z}")

    def dest_callback(self, msg):
        self.dest = msg.data
        rospy.loginfo(f"Destination bin saved: {self.dest}")

    def wait_for_initial_push_point(self):
        rospy.loginfo("Waiting for initial push point...")
        while not rospy.is_shutdown() and self.initial_push_point is None:
            sleep(0.1)
        return self.initial_push_point

    def wait_for_side_point_and_dest(self):
        rospy.loginfo("Waiting for side point and dest...")
        while not rospy.is_shutdown() and (self.side_point is None or self.dest is None):
            sleep(0.1)
        return self.side_point, self.dest

    def get_current_push_point(self):
        return self.current_push_point


if __name__ == "__main__":
    rospy.init_node("position_manager", anonymous=True)
    position_manager = PositionManager()

    while not rospy.is_shutdown():
        initial_push_point = position_manager.wait_for_initial_push_point()
        rospy.loginfo(f"received {initial_push_point=}")
        side_point = position_manager.wait_for_side_point()
        rospy.loginfo(f"received {side_point=}")
        
        position_manager.reset() # TODO: call after the box goes in the bin
