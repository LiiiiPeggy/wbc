#!/usr/bin/env python3
# ################################
# Python: fake whole-body system with write counters begin
# ################################
from __future__ import print_function

import threading

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from remani_real_msgs.msg import Cr10Status
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt32
from std_msgs.msg import UInt32MultiArray


class FakeWholeBodySystem(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._ranger_cmds = 0
        self._arm_goals = 0
        self._arm_cancels = 0
        self._arm_stops = 0
        self._non_hold_samples = 0

        self._odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self._raw_joint_pub = rospy.Publisher(
            "/remani/cr10_joint_states_raw", JointState, queue_size=1)
        self._status_pub = rospy.Publisher(
            "/remani/cr10_status", Cr10Status, queue_size=1)
        self._watchdog_ready_pub = rospy.Publisher(
            "/remani/ranger_watchdog_ready", Bool, queue_size=1, latch=True)
        self._watchdog_timeout_pub = rospy.Publisher(
            "/remani/ranger_watchdog_timed_out", Bool, queue_size=1, latch=True)
        self._counts_pub = rospy.Publisher(
            "/test/write_counts", UInt32MultiArray, queue_size=1, latch=True)
        self._goal_count_pub = rospy.Publisher(
            "/remani/test/cr10_action_goal_count", UInt32, queue_size=1, latch=True)

        # Sink for any accidental hardware Ranger publishes.
        rospy.Subscriber(
            "/remani/hardware/ranger/cmd_vel", Twist, self._on_ranger, queue_size=10)

        self._action_server = actionlib.SimpleActionServer(
            "/cr10_robot/joint_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._on_action_goal,
            auto_start=False)
        self._action_server.register_preempt_callback(self._on_preempt)
        self._action_server.start()

        self._watchdog_ready_pub.publish(Bool(data=True))
        self._watchdog_timeout_pub.publish(Bool(data=False))
        self._publish_counts()
        self._timer = rospy.Timer(rospy.Duration(0.05), self._on_timer)

    def _publish_counts(self):
        with self._lock:
            msg = UInt32MultiArray()
            msg.data = [
                self._ranger_cmds,
                self._arm_goals,
                self._arm_cancels,
                self._arm_stops,
                self._non_hold_samples,
            ]
            goals = self._arm_goals
        self._counts_pub.publish(msg)
        self._goal_count_pub.publish(UInt32(data=goals))

    def _on_ranger(self, _msg):
        with self._lock:
            self._ranger_cmds += 1
        self._publish_counts()

    def _on_preempt(self):
        with self._lock:
            self._arm_cancels += 1
        self._publish_counts()
        if self._action_server.is_active():
            self._action_server.set_preempted(FollowJointTrajectoryResult())

    def _on_action_goal(self, goal):
        with self._lock:
            self._arm_goals += 1
            # Count non-hold samples as points after the first two hold points.
            if goal is not None and goal.trajectory.points:
                pts = goal.trajectory.points
                if len(pts) > 2:
                    self._non_hold_samples += len(pts) - 2
        self._publish_counts()
        # Dry-run must never send; abort if a goal somehow arrives.
        self._action_server.set_aborted(FollowJointTrajectoryResult())

    def _on_timer(self, _event):
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
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
    rospy.init_node("fake_whole_body_system")
    FakeWholeBodySystem()
    rospy.loginfo("fake_whole_body_system online")
    rospy.spin()


if __name__ == "__main__":
    main()
# ################################
# Python: fake whole-body system with write counters end
# ################################
