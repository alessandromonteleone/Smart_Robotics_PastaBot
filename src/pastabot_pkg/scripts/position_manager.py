#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from time import sleep

class PositionManager:
    def __init__(self):
        rospy.init_node("position_manager", anonymous=True)
        self.push_point_sub = rospy.Subscriber("box/push_point", Point, self.push_point_callback)
        self.side_point_sub = rospy.Subscriber("box/side_point", Point, self.side_point_callback)

        self.initial_push_point = None
        self.current_push_point = None
        self.side_point = None

    def reset(self):
        self.initial_push_point = None
        self.current_push_point = None
        self.side_point = None

    def push_point_callback(self, msg):
        if self.initial_push_point is None:
            self.initial_push_point = msg
            rospy.loginfo(f"Initial push point saved: x={msg.x}, y={msg.y}, z={msg.z}")
        self.current_push_point = msg

    def side_point_callback(self, msg):
        self.side_point = msg
        rospy.loginfo(f"Side point saved: x={msg.x}, y={msg.y}, z={msg.z}")

    def wait_for_initial_push_point(self):
        rospy.loginfo("Waiting for initial push point...")
        while not rospy.is_shutdown() and self.initial_push_point is None:
            sleep(0.1)
        return self.initial_push_point

    def wait_for_side_point(self):
        rospy.loginfo("Waiting for side point...")
        while not rospy.is_shutdown() and self.side_point is None:
            sleep(0.1)
        return self.side_point

    def get_current_push_point(self):
        return self.current_push_point

    # def run(self):
    #     rospy.spin()


if __name__ == "__main__":
    position_manager = PositionManager()
    initial_push_point = position_manager.wait_for_initial_push_point()
    rospy.loginfo(f"received {initial_push_point=}")
    side_point = position_manager.wait_for_side_point()
    rospy.loginfo(f"received {side_point=}")
