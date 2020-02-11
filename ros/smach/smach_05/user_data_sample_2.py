#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import smach
import smach_ros


class GetObjectPositionDummy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok', 'ng'],
                             input_keys=['object_name'],
                             output_keys=['x', 'y', 'z'])
        self.object_positions = {'コーヒー': [0.6, 0.2, 0.7], '紅茶': [
            0.5, -0.2, 0.75], 'オレンジジュース': [0.8, -0.15, 1.5]}

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        rospy.loginfo('Target = %s' % userdata.object_name)
        pos = self.object_positions[userdata.object_name]
        rospy.loginfo('%s の座標は %.2f, %.2f, %.2f' % (userdata.object_name, pos[0], pos[1], pos[2]))
        userdata.x = pos[0]
        userdata.y = pos[1]
        userdata.z = pos[2]
        return 'ok'


class GrabItemDummy(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['ok', 'ng'], input_keys=['x', 'y', 'z'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        rospy.loginfo('Target = %f, %f, %f' %
                      (userdata.x, userdata.y, userdata.z))
        rospy.loginfo('把持に成功。')
        return 'ok'


def main_1():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])
    sm.userdata.x = 0.7
    sm.userdata.y = 0.05
    sm.userdata.z = 0.73

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GrabItemDummy_1', GrabItemDummy(),
                               transitions={'ok': 'OK', 'ng': 'NG'})

    sis = smach_ros.IntrospectionServer('state_machine_simple', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

def main_2():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])
    sm.userdata.object_name = 'コーヒー'

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GetObjectPositionDummy_1', GetObjectPositionDummy(),
                               transitions={'ok': 'GrabItemDummy_1', 'ng': 'NG'})
        smach.StateMachine.add('GrabItemDummy_1', GrabItemDummy(),
                               transitions={'ok': 'OK', 'ng': 'NG'})

    sis = smach_ros.IntrospectionServer('state_machine_simple', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main_1()
