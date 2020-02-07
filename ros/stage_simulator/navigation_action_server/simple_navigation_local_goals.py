#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def main():
    rospy.init_node('simple_navigation_goals')
    # Action Client
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
    while not ac.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Waiting for the move_base action server to come up")

    rospy.loginfo("The server comes up")

    # ゴールの生成
    coord_type = "base_link"  # ロボットローカル座標系
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = coord_type
    goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻

    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal")
    ac.send_goal(goal)
    finished = ac.wait_for_result(rospy.Duration(30))
    state = ac.get_state()
    if finished:
        rospy.loginfo("Finished: (%d)", state)
    else:
        rospy.loginfo("Time out: (%d)", state)


if __name__ == '__main__':
    main()
