#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import threading

import nav_msgs.msg
import rospy
import sensor_msgs.msg
import smach
import smach_ros


class SleepState(smach.State):
    def __init__(self, time_limit):
        smach.State.__init__(self, outcomes=['ok'])
        self.time_limit = time_limit

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__ +
                      '. Sleeping ' + str(self.time_limit) + ' secs')
        rospy.sleep(self.time_limit)
        return 'ok'


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1.0):
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type

    def get_msg(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e)
        return message


class WaitForLaserScan(smach.State):
    def __init__(self, topic='/base_scan', time_limit=None, msg_wait=1.0):
        smach.State.__init__(self, outcomes=['ok', 'ng'])
        self.sensor_msg = SensorMessageGetter(
            topic, sensor_msgs.msg.LaserScan, msg_wait)
        self.time_limit = time_limit
        self.end_time = None

    def is_time_limit(self):
        if self.time_limit is None or self.end_time is None:
            return False
        return rospy.Time.now() >= self.end_time

    def execute(self, userdata):
        self.end_time = None
        if self.time_limit is not None:
            self.end_time = rospy.Time.now() + rospy.Duration.from_sec(self.time_limit)
        while self.is_time_limit() is False:
            msg = self.sensor_msg.get_msg()
            if msg is not None:
                rospy.loginfo('Recv sensor. frame_id = ' +
                              msg.header.frame_id + ', seq = ' + str(msg.header.seq))
                # http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html をよく見よう。
                # len(msg.ranges) がスキャンデータの個数
                # msg.ranges[i] で i 番目のセンサデータが分かる。
        return 'ng'


class WaitForOdometry(smach.State):
    def __init__(self, topic='/odom', time_limit=None, msg_wait=1.0):
        smach.State.__init__(self, outcomes=['ok', 'ng'])
        self.sensor_msg = SensorMessageGetter(
            topic, nav_msgs.msg.Odometry, msg_wait)
        self.time_limit = time_limit
        self.end_time = None

    def is_time_limit(self):
        if self.time_limit is None or self.end_time is None:
            return False
        return rospy.Time.now() >= self.end_time

    def execute(self, userdata):
        self.end_time = None
        if self.time_limit is not None:
            self.end_time = rospy.Time.now() + rospy.Duration.from_sec(self.time_limit)
        while self.is_time_limit() is False:
            msg = self.sensor_msg.get_msg()
            if msg is not None:
                rospy.loginfo('Recv sensor. frame_id = ' +
                              msg.header.frame_id + ', seq = ' + str(msg.header.seq))
                # http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html をよく見よう。
                # msg.pose.pose.position.x などで座標が分かる。
        return 'ng'


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Sleep', SleepState(2.0),
                               transitions={'ok': 'WaitForLaserScan'})
        smach.StateMachine.add('WaitForLaserScan', WaitForLaserScan('/base_scan', 3.0),
                               transitions={'ok': 'WaitForOdometry',
                                            'ng': 'WaitForOdometry'})
        smach.StateMachine.add('WaitForOdometry', WaitForOdometry('/odom',2.0),
                               transitions={'ok': 'OK',
                                            'ng': 'NG'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
