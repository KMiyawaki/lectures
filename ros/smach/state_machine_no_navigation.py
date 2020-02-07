#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import actionlib
import smach
import smach_ros
from geometry_msgs.msg import Quaternion, Twist


class GoStraightByTime(smach.State):
    def __init__(self, time_limit, linear_vel=0.4, cmd_vel="/cmd_vel"):
        smach.State.__init__(self, outcomes=['ok'])
        self.cmd_vel = cmd_vel
        self.linear_vel = linear_vel
        self.time_limit = time_limit

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        pub = rospy.Publisher(self.cmd_vel, Twist, queue_size=10)
        vel = Twist()
        vel.linear.x = self.linear_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        start_time = rospy.get_time()
        diff = rospy.get_time() - start_time
        while diff < self.time_limit:
            pub.publish(vel)
            rospy.sleep(0.1)
            diff = rospy.get_time() - start_time
        vel.linear.x = 0.0
        pub.publish(vel)
        return 'ok'


def main():
    rospy.init_node('smach_example_state_machine')
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('go_straight_01', GoStraightByTime(5.0),
                               transitions={'ok': 'OK'})

    sis = smach_ros.IntrospectionServer('state_machine_simple', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
