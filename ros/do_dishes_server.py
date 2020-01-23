#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from beginner_tutorials.msg import *


class DoDishesServer:
    WASHERS = [["N/A", -1], ["MITSUBISHI TK-TS5-W", 25],
               ["TOSHIBA VD-B10S-LK", 30], ["PANASONIC NP-TCB4-W", 41]]

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'do_dishes', DoDishesAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        rospy.loginfo("Accept request. Type = " + type(goal).__name__ +
                      ", dishwasher_id = " + str(goal.dishwasher_id))
        washer = DoDishesServer.WASHERS[goal.dishwasher_id]
        rospy.loginfo(washer[0] + " started to wash.")
        for i in range(3):
            rospy.loginfo("Please wait...")
            rospy.sleep(1.0)
        result = DoDishesResult()
        result.total_dishes_cleaned = washer[1]
        # Do lots of awesome groundbreaking robot stuff here
        self.server.set_succeeded(result)
        rospy.loginfo("Sent result.")


if __name__ == '__main__':
    node_name = "do_dishes_server"
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " node started. Waiting for action client.")
    server = DoDishesServer()
    rospy.spin()
