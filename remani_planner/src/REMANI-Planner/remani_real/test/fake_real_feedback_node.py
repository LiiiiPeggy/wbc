#!/usr/bin/env python3
# ################################
# Python: fake real feedback node begin
# ################################
"""Publish fake Ranger/CR10 feedback and a rejecting CR10 action server for dry-run rostest."""

from __future__ import print_function

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from remani_real_msgs.msg import Cr10Status
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt32


class FakeRealFeedbackNode(object):
    def __init__(self):
        self._goal_count = 0
        self._count_pub = rospy.Publisher(
            "/remani/test/cr10_action_goal_count", UInt32, queue_size=1, latch=True)
        self._odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self._raw_joint_pub = rospy.Publisher(
            "/remani/cr10_joint_states_raw", JointState, queue_size=1)
        self._status_pub = rospy.Publisher(
            "/remani/cr10_status", Cr10Status, queue_size=1)
        self._watchdog_ready_pub = rospy.Publisher(
            "/remani/ranger_watchdog_ready", Bool, queue_size=1, latch=True)
        self._watchdog_timeout_pub = rospy.Publisher(
            "/remani/ranger_watchdog_timed_out", Bool, queue_size=1, latch=True)

        self._action_server = actionlib.SimpleActionServer(
            "/cr10_robot/joint_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._on_action_goal,
            auto_start=False)
        self._action_server.start()

        self._watchdog_ready_pub.publish(Bool(data=True))
        self._watchdog_timeout_pub.publish(Bool(data=False))
        self._publish_goal_count()

        self._timer = rospy.Timer(rospy.Duration(0.05), self._on_timer)

    def _publish_goal_count(self):
        self._count_pub.publish(UInt32(data=self._goal_count))

    def _on_action_goal(self, _goal):
        self._goal_count += 1
        self._publish_goal_count()
        self._action_server.set_aborted(FollowJointTrajectoryResult())

    def _on_timer(self, _event):
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self._odom_pub.publish(odom)

        raw = JointState()
        raw.header.stamp = stamp
        raw.name = ["joint3", "joint1", "joint2", "joint6", "joint4", "joint5"]
        raw.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._raw_joint_pub.publish(raw)

        status = Cr10Status()
        status.header.stamp = stamp
        status.connected = True
        status.enabled = True
        status.error_status = 0
        status.robot_mode = 5
        self._status_pub.publish(status)


def main():
    rospy.init_node("fake_real_feedback")
    FakeRealFeedbackNode()
    rospy.loginfo("fake_real_feedback_node online (no hardware cmd_vel or write services)")
    rospy.spin()


if __name__ == "__main__":
    main()
# ################################
# Python: fake real feedback node end
# ################################
