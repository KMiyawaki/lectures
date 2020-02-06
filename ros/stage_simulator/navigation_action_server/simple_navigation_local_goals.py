#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == '__main__':
    rospy.init_node('simple_navigation_goals')
    # Action Client
    AC = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
    while not AC.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Waiting for the move_base action server to come up")

    rospy.loginfo("The server comes up")

    # ゴールの生成
    COORD_TYPE = "base_link" # ロボットローカル座標系
    GOAL = MoveBaseGoal()
    GOAL.target_pose.header.frame_id = COORD_TYPE
    GOAL.target_pose.header.stamp = rospy.Time.now() # 現在時刻

    GOAL.target_pose.pose.position.x = 1.0
    GOAL.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal")
    AC.send_goal(GOAL)
    FINISHED = AC.wait_for_result(rospy.Duration(30))
    STATE = AC.get_state()
    if FINISHED:
        rospy.loginfo("Finished: (%d)", STATE)
    else:
        rospy.loginfo("Time out: (%d)", STATE)
