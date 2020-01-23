#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import actionlib_msgs
from actionlib_msgs.msg import *
from beginner_tutorials.msg import *

if __name__ == '__main__':
    rospy.init_node("do_dishes_client")
    action_server = "do_dishes"
    client = actionlib.SimpleActionClient(action_server, DoDishesAction)
    # アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒)
    while not client.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Waiting for the action server to come up")
        client.wait_for_server()
    rospy.loginfo("Connected to for the " + action_server + " to come up")
    goal = DoDishesGoal()
    # goal.dishwasher_id = 1
    goal.dishwasher_id = 2
    # goal.dishwasher_id = 3
    # Fill in the goal here
    client.send_goal(goal)
    while not rospy.is_shutdown():
        client.wait_for_result(rospy.Duration.from_sec(5.0))
        state = client.get_state()
        # http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Scceeded: state = " + str(state))
            result = client.get_result()
            rospy.loginfo("Recv result. Type = " + type(result).__name__ + ", total_dishes_cleaned = " +
                        str(result.total_dishes_cleaned))
        else:
            rospy.loginfo("Failed: state = " + str(state))
        break
