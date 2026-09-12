#!/usr/bin/env python3
# ################################
# Python: real control plane rostest begin
# ################################
"""Rostest: Phase-2 dry-run Plan -> PLANNED -> Execute -> SUCCEEDED with zero hardware output."""

from __future__ import print_function

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from remani_real_msgs.msg import ExecutionState
from remani_real_msgs.srv import ExecuteCandidate
from std_msgs.msg import Empty, UInt32


class RealControlPlaneTest(unittest.TestCase):
    _READY_TIMEOUT = 30.0
    _PLANNED_TIMEOUT = 30.0
    _EXECUTE_TIMEOUT = 30.0
    _PREVIEW_TIMEOUT = 10.0

    def setUp(self):
        self._lock = threading.Lock()
        self._execution_state = None
        self._preview_count = 0
        self._goal_count = None

        self._state_sub = rospy.Subscriber(
            "/remani/execution_state", ExecutionState, self._on_execution_state, queue_size=10)
        self._preview_sub = rospy.Subscriber(
            "/remani/dry_run/ranger_cmd_vel_preview", Twist, self._on_preview, queue_size=10)
        self._goal_count_sub = rospy.Subscriber(
            "/remani/test/cr10_action_goal_count", UInt32, self._on_goal_count, queue_size=1)
        self._plan_pub = rospy.Publisher("/ee_goal_plan", Empty, queue_size=1)

        rospy.sleep(1.0)

    def _on_execution_state(self, msg):
        with self._lock:
            self._execution_state = msg

    def _on_preview(self, _msg):
        with self._lock:
            self._preview_count += 1

    def _on_goal_count(self, msg):
        with self._lock:
            self._goal_count = msg.data

    def _latest_state(self):
        with self._lock:
            return self._execution_state

    def _preview_messages(self):
        with self._lock:
            return self._preview_count

    def _latest_goal_count(self):
        with self._lock:
            return self._goal_count

    @staticmethod
    def _readiness_ok(msg):
        return (
            msg.odom_ready
            and msg.cr10_joint_ready
            and msg.cr10_velocity_valid
            and msg.tf_ready
            and msg.robot_status_ready
            and msg.action_server_ready
            and msg.grid_map_ready
            and msg.ranger_watchdog_ready
            and not msg.ranger_watchdog_timed_out
        )

    @staticmethod
    def _ready_ish(msg):
        return (
            msg.executor_state == ExecutionState.EXECUTOR_NONE
            and RealControlPlaneTest._readiness_ok(msg)
        )

    def _wait_for(self, predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            msg = self._latest_state()
            if msg is not None and predicate(msg):
                return msg
            rospy.sleep(0.05)
        self.fail("Timed out waiting for {0}".format(description))

    @staticmethod
    def _published_topics():
        # rospy helper avoids Master API import quirks under rostest wrappers.
        return {name for name, _type in rospy.get_published_topics()}

    def test_dry_run_happy_path(self):
        self._wait_for(self._ready_ish, self._READY_TIMEOUT, "Ready-ish execution_state")

        self._plan_pub.publish(Empty())
        planned = self._wait_for(
            lambda msg: msg.candidate_valid
            and msg.executor_state == ExecutionState.EXECUTOR_PLANNED
            and msg.candidate_id > 0,
            self._PLANNED_TIMEOUT,
            "candidate_valid and EXECUTOR_PLANNED")
        candidate_id = planned.candidate_id

        published = self._published_topics()
        self.assertNotIn("/remani/hardware/ranger/cmd_vel", published)

        execute = rospy.ServiceProxy("/remani/execute", ExecuteCandidate)
        execute.wait_for_service(timeout=self._READY_TIMEOUT)

        stale = execute(candidate_id - 1)
        self.assertFalse(stale.accepted)

        accepted = execute(candidate_id)
        self.assertTrue(accepted.accepted)

        self._wait_for(
            lambda msg: msg.executor_state == ExecutionState.EXECUTOR_SUCCEEDED,
            self._EXECUTE_TIMEOUT,
            "EXECUTOR_SUCCEEDED")

        published = self._published_topics()
        self.assertNotIn("/remani/hardware/ranger/cmd_vel", published)

        goal_count = self._latest_goal_count()
        self.assertIsNotNone(goal_count)
        self.assertEqual(0, goal_count)

        deadline = rospy.Time.now() + rospy.Duration(self._PREVIEW_TIMEOUT)
        while rospy.Time.now() < deadline and self._preview_messages() == 0:
            rospy.sleep(0.05)
        self.assertGreater(self._preview_messages(), 0)

    # Protocol-fault injection (ADD-before-START, duplicate ADD, skipped ID,
    # timeout, START during EXECUTING) is documented in the control-plane plan
    # but intentionally skipped here to keep the rostest deterministic.


if __name__ == "__main__":
    rospy.init_node("test_real_control_plane")
    rostest.rosrun("remani_real", "test_real_control_plane", RealControlPlaneTest)
# ################################
# Python: real control plane rostest end
# ################################
