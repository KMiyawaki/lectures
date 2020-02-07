#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import roslib
import rospy
import actionlib
import actionlib_msgs
import smach
import smach_ros
import tf
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler


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


class CheckWayPoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok', 'ng'], input_keys=[
                             'waypoints'], output_keys=['target_pose'])
        self.crnt_idx = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        waypoints = userdata.waypoints
        if self.crnt_idx >= len(waypoints):
            return 'ok'
        else:
            wp = waypoints[self.crnt_idx]
            self.crnt_idx = self.crnt_idx + 1
            wp.header.stamp = rospy.Time.now()
            userdata.target_pose = wp
            return 'ng'


class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok', 'ng'], input_keys=[
                             'angle'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        yaw_pre = 0.0
        angle_traveled = 0.0
        angle = userdata.angle
        try:
            yaw_pre = self.__get_yaw()
        except Exception as e:
            rospy.logerr(e)
            return 'ng'
        while math.fabs(angle_traveled) < math.fabs(angle):
            yaw = 0.0
            try:
                yaw = self.__get_yaw()
            except Exception as e:
                rospy.logerr(e)
                return 'ng'
            angle_traveled = angle_traveled + (yaw - yaw_pre)
            yaw_pre = yaw

    def __get_yaw(self):
        try:
            listener = tf.TransformListener()
            listener.waitForTransform(
                'map', 'base_link', rospy.Time(), rospy.Duration(4.0))
            (_, rot) = listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            (_, _, yaw) = euler_from_quaternion(rot)
            return yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            raise e


class TurnToPoint(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['ok', 'ng'], input_keys=['target_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        try:
            listener = tf.TransformListener()
            listener.waitForTransform(
                'map', 'base_link', rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            x = trans[0]
            y = trans[1]
            (_, _, yaw) = euler_from_quaternion(rot)
            rospy.loginfo("Global Robot Position (%.2f, %.2f) yaw = %.2f degree" %
                          (x, y, math.degrees(yaw)))
            rospy.loginfo("Target Point ( % .2f, % .2f)" %
                          (userdata.target_pose.pose.position.x, userdata.target_pose.pose.position.y))
            # x, y, yaw と userdata.target_pose.position.x, userdata.target_pose.position.y をもとに
            # ロボットを userdata.target_pose.pose.position.x, userdata.target_pose.pose.position.y に回転させる。
            return 'ok'
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(str(e))
            return 'ng'


def waypoint_navigation():
    sm = smach.StateMachine(outcomes=['ok', 'ng'], input_keys=['waypoints'])
    with sm:
        smach.StateMachine.add('CheckWayPoints', CheckWayPoints(),
                               transitions={'ok': 'ok', 'ng': 'TurnToPoint'})
        smach.StateMachine.add('TurnToPoint', TurnToPoint(),
                               transitions={'ok': 'call_move_base', 'ng': 'ng'})
        smach.StateMachine.add('call_move_base',
                               SimpleActionState('move_base', MoveBaseAction,
                                                 goal_slots=['target_pose']),
                               transitions={'succeeded': 'CheckWayPoints', 'preempted': 'ng', 'aborted': 'ng'})
    return sm


def main():
    rospy.init_node('smach_example_state_machine')
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])
    wp = PoseStamped()
    wp.header.frame_id = "map"
    wp.header.stamp = rospy.Time.now()
    wp.pose.position.x = -4.0
    wp.pose.position.y = -0.3
    wp.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0, 0, math.radians(-170))
    wp.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    sm.userdata.waypoints = [wp]

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GoStraightByTime', GoStraightByTime(1.0),
                               transitions={'ok': 'WayPointNavigation'})
        smach.StateMachine.add('WayPointNavigation', waypoint_navigation(),
                               transitions={'ok': 'OK', 'ng': 'NG'})

    sis = smach_ros.IntrospectionServer('state_machine_simple', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
