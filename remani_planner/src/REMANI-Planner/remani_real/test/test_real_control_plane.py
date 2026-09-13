#!/usr/bin/env python3
# ################################
# Python: real control plane rostest begin
# ################################
"""Rostest: dry-run happy path plus Gate protocol-fault injection."""

from __future__ import print_function

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from quadrotor_msgs.msg import PolynomialTraj
from remani_real_msgs.msg import ExecutionState
from remani_real_msgs.srv import ExecuteCandidate
from std_msgs.msg import Empty, String, UInt32
from std_srvs.srv import Trigger


class RealControlPlaneTest(unittest.TestCase):
    _READY_TIMEOUT = 30.0
    _PLANNED_TIMEOUT = 30.0
    _EXECUTE_TIMEOUT = 30.0
    _PREVIEW_TIMEOUT = 10.0
    _FAULT_TIMEOUT = 10.0

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
        self._scenario_pub = rospy.Publisher(
            "/remani/test/planner_scenario", String, queue_size=1, latch=True)
        self._candidate_pub = rospy.Publisher(
            "/remani/planner_candidate", PolynomialTraj, queue_size=10)

        rospy.sleep(1.0)
        self._set_scenario("happy")
        self._recover_to_ready()

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

    def _set_scenario(self, scenario):
        self._scenario_pub.publish(String(data=scenario))
        rospy.sleep(0.2)

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

    @staticmethod
    def _can_plan(msg):
        # Ready or Succeeded both allow a new Plan under the deployment SM.
        return (
            RealControlPlaneTest._readiness_ok(msg)
            and msg.executor_state in (
                ExecutionState.EXECUTOR_NONE,
                ExecutionState.EXECUTOR_SUCCEEDED,
            )
        )

    def _wait_for(self, predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            msg = self._latest_state()
            if msg is not None and predicate(msg):
                return msg
            rospy.sleep(0.05)
        self.fail("Timed out waiting for {0}".format(description))

    def _abort(self):
        abort = rospy.ServiceProxy("/remani/abort", Trigger)
        abort.wait_for_service(timeout=self._READY_TIMEOUT)
        return abort()

    def _recover_to_ready(self):
        # ################################
        # Python: clear Planning/Error/motion before next case begin
        # ################################
        msg = self._latest_state()
        if msg is not None:
            if msg.executor_state == ExecutionState.EXECUTOR_ERROR:
                self._abort()
            elif msg.transaction_state == ExecutionState.TRANSACTION_ASSEMBLING:
                self._abort()
            elif msg.executor_state in (
                    ExecutionState.EXECUTOR_PLANNED,
                    ExecutionState.EXECUTOR_EXECUTING,
                    ExecutionState.EXECUTOR_PAUSED):
                self._abort()
        self._wait_for(self._can_plan, self._READY_TIMEOUT, "plan-capable execution_state")
        # ################################
        # Python: clear Planning/Error/motion before next case end
        # ################################

    @staticmethod
    def _published_topics():
        # rospy helper avoids Master API import quirks under rostest wrappers.
        return {name for name, _type in rospy.get_published_topics()}

    def _assert_zero_hardware(self):
        published = self._published_topics()
        self.assertNotIn("/remani/hardware/ranger/cmd_vel", published)
        goal_count = self._latest_goal_count()
        self.assertIsNotNone(goal_count)
        self.assertEqual(0, goal_count)

    def test_dry_run_happy_path(self):
        self._set_scenario("happy")
        self._plan_pub.publish(Empty())
        planned = self._wait_for(
            lambda msg: msg.candidate_valid
            and msg.executor_state == ExecutionState.EXECUTOR_PLANNED
            and msg.candidate_id > 0,
            self._PLANNED_TIMEOUT,
            "candidate_valid and EXECUTOR_PLANNED")
        candidate_id = planned.candidate_id

        self._assert_zero_hardware()

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

        self._assert_zero_hardware()

        deadline = rospy.Time.now() + rospy.Duration(self._PREVIEW_TIMEOUT)
        while rospy.Time.now() < deadline and self._preview_messages() == 0:
            rospy.sleep(0.05)
        self.assertGreater(self._preview_messages(), 0)

    def test_add_before_start_keeps_planning(self):
        # ################################
        # Python: ADD-before-START stays Planning (idle reject ignored) begin
        # ################################
        self._set_scenario("add_before_start")
        self._plan_pub.publish(Empty())
        self._wait_for(
            lambda msg: msg.transaction_state == ExecutionState.TRANSACTION_ASSEMBLING,
            self._FAULT_TIMEOUT,
            "Planning/ASSEMBLING after Plan")
        rospy.sleep(0.5)
        state = self._latest_state()
        self.assertEqual(ExecutionState.EXECUTOR_NONE, state.executor_state)
        self.assertEqual(ExecutionState.TRANSACTION_ASSEMBLING, state.transaction_state)
        self.assertNotEqual(ExecutionState.EXECUTOR_ERROR, state.executor_state)
        self._assert_zero_hardware()
        # ################################
        # Python: ADD-before-START stays Planning (idle reject ignored) end
        # ################################

    def test_duplicate_add_enters_error(self):
        # ################################
        # Python: duplicate ADD -> SEGMENT_SEQUENCE ERROR begin
        # ################################
        self._set_scenario("duplicate_add")
        self._plan_pub.publish(Empty())
        err = self._wait_for(
            lambda msg: msg.executor_state == ExecutionState.EXECUTOR_ERROR
            and msg.last_error_code == "SEGMENT_SEQUENCE",
            self._FAULT_TIMEOUT,
            "EXECUTOR_ERROR SEGMENT_SEQUENCE for duplicate ADD")
        self.assertEqual("SEGMENT_SEQUENCE", err.last_error_code)
        self._assert_zero_hardware()
        # ################################
        # Python: duplicate ADD -> SEGMENT_SEQUENCE ERROR end
        # ################################

    def test_skipped_id_enters_error(self):
        # ################################
        # Python: skipped ADD id -> SEGMENT_SEQUENCE ERROR begin
        # ################################
        self._set_scenario("skipped_id")
        self._plan_pub.publish(Empty())
        err = self._wait_for(
            lambda msg: msg.executor_state == ExecutionState.EXECUTOR_ERROR
            and msg.last_error_code == "SEGMENT_SEQUENCE",
            self._FAULT_TIMEOUT,
            "EXECUTOR_ERROR SEGMENT_SEQUENCE for skipped id")
        self.assertEqual("SEGMENT_SEQUENCE", err.last_error_code)
        self._assert_zero_hardware()
        # ################################
        # Python: skipped ADD id -> SEGMENT_SEQUENCE ERROR end
        # ################################

    def test_assembly_timeout_enters_error(self):
        # ################################
        # Python: hang after ADD -> ASSEMBLY_TIMEOUT ERROR begin
        # ################################
        self._set_scenario("hang_after_add")
        self._plan_pub.publish(Empty())
        err = self._wait_for(
            lambda msg: msg.executor_state == ExecutionState.EXECUTOR_ERROR
            and msg.last_error_code == "ASSEMBLY_TIMEOUT",
            self._FAULT_TIMEOUT,
            "EXECUTOR_ERROR ASSEMBLY_TIMEOUT")
        self.assertEqual("ASSEMBLY_TIMEOUT", err.last_error_code)
        self._assert_zero_hardware()
        # ################################
        # Python: hang after ADD -> ASSEMBLY_TIMEOUT ERROR end
        # ################################

    def test_start_during_executing_ignored(self):
        # ################################
        # Python: START during EXECUTING leaves frozen candidate begin
        # ################################
        self._set_scenario("happy")
        self._plan_pub.publish(Empty())
        planned = self._wait_for(
            lambda msg: msg.candidate_valid
            and msg.executor_state == ExecutionState.EXECUTOR_PLANNED
            and msg.candidate_id > 0,
            self._PLANNED_TIMEOUT,
            "PLANNED before execute")
        candidate_id = planned.candidate_id

        execute = rospy.ServiceProxy("/remani/execute", ExecuteCandidate)
        execute.wait_for_service(timeout=self._READY_TIMEOUT)
        self.assertTrue(execute(candidate_id).accepted)

        self._wait_for(
            lambda msg: msg.executor_state == ExecutionState.EXECUTOR_EXECUTING,
            self._EXECUTE_TIMEOUT,
            "EXECUTOR_EXECUTING")

        start = PolynomialTraj()
        start.header.stamp = rospy.Time.now()
        start.action = PolynomialTraj.ACTION_WARN_START
        start.trajectory_id = 0
        self._candidate_pub.publish(start)
        rospy.sleep(0.3)

        state = self._latest_state()
        self.assertIn(
            state.executor_state,
            (ExecutionState.EXECUTOR_EXECUTING, ExecutionState.EXECUTOR_SUCCEEDED))
        if state.executor_state == ExecutionState.EXECUTOR_EXECUTING:
            self.assertEqual(candidate_id, state.candidate_id)
        self._assert_zero_hardware()
        # ################################
        # Python: START during EXECUTING leaves frozen candidate end
        # ################################


if __name__ == "__main__":
    rospy.init_node("test_real_control_plane")
    rostest.rosrun("remani_real", "test_real_control_plane", RealControlPlaneTest)
# ################################
# Python: real control plane rostest end
# ################################
