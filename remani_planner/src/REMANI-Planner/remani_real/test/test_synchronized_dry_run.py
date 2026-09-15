#!/usr/bin/env python3
# ################################
# Python: synchronized dry-run rostest begin
# ################################
from __future__ import print_function

import threading
import unittest

import rosgraph
import rospy
import rostest
from remani_real_msgs.msg import ExecutionState
from remani_real_msgs.srv import ExecuteCandidate
from std_msgs.msg import Empty, UInt32MultiArray


def publishers(topic):
    published, _, _ = rosgraph.Master("/synchronized_dry_run").getSystemState()
    for name, nodes in published:
        if name == topic:
            return nodes
    return []


class SynchronizedDryRunTest(unittest.TestCase):
    _TIMEOUT = 40.0

    def setUp(self):
        self._lock = threading.Lock()
        self._state = None
        self._counts = None
        rospy.Subscriber(
            "/remani/execution_state", ExecutionState, self._on_state, queue_size=20)
        rospy.Subscriber(
            "/test/write_counts", UInt32MultiArray, self._on_counts, queue_size=5)
        self._plan_pub = rospy.Publisher("/ee_goal_plan", Empty, queue_size=1)
        rospy.sleep(1.0)
        self._wait_ready()

    def _on_state(self, msg):
        with self._lock:
            self._state = msg

    def _on_counts(self, msg):
        with self._lock:
            self._counts = list(msg.data)

    def _latest(self):
        with self._lock:
            return self._state

    def _write_counts(self):
        with self._lock:
            return self._counts

    def _wait_for(self, predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            msg = self._latest()
            if msg is not None and predicate(msg):
                return msg
            rospy.sleep(0.05)
        self.fail("Timed out waiting for {0}".format(description))

    def _wait_ready(self):
        self._wait_for(
            lambda m: m.odom_ready and m.cr10_joint_ready and m.action_server_ready,
            self._TIMEOUT,
            "readiness",
        )

    def _assert_zero_writes(self):
        self.assertEqual(0, len(publishers("/remani/hardware/ranger/cmd_vel")))
        counts = self._write_counts()
        self.assertIsNotNone(counts)
        self.assertEqual([0, 0, 0, 0, 0], counts)

    def test_synchronized_dry_run_sequence(self):
        self._assert_zero_writes()
        self._plan_pub.publish(Empty())
        planned = self._wait_for(
            lambda m: m.candidate_valid
            and m.executor_state == ExecutionState.EXECUTOR_PLANNED
            and m.candidate_id > 0,
            self._TIMEOUT,
            "PLANNED",
        )
        candidate_id = planned.candidate_id

        execute = rospy.ServiceProxy("/remani/execute", ExecuteCandidate)
        execute.wait_for_service(timeout=self._TIMEOUT)
        t_request = rospy.get_time()
        self.assertTrue(execute(candidate_id).accepted)

        executing = self._wait_for(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_EXECUTING
            and m.requested_t0 > 0.0,
            self._TIMEOUT,
            "EXECUTING with requested_t0",
        )
        # requested_t0 is SteadyTime seconds; lead is relative — compare delta via
        # ranger_trajectory_start_time once available, and assert lead ≈ 1.0 from
        # execute request wall approximation using progress==0 hold.
        rospy.sleep(0.2)
        hold = self._latest()
        self.assertEqual(ExecutionState.EXECUTOR_EXECUTING, hold.executor_state)
        self.assertLess(hold.execution_progress, 0.05)

        # Wait until post-T0 progress advances, then succeed.
        self._wait_for(
            lambda m: m.execution_progress > 0.05
            or m.executor_state == ExecutionState.EXECUTOR_SUCCEEDED,
            self._TIMEOUT,
            "post-T0 progress",
        )
        lead_proxy = rospy.get_time() - t_request
        # Lead should be near 1.0 before motion progress; allow wide CI jitter.
        self.assertGreaterEqual(lead_proxy, 0.7)

        succeeded = self._wait_for(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_SUCCEEDED,
            self._TIMEOUT,
            "SUCCEEDED",
        )
        self.assertAlmostEqual(1.0, succeeded.execution_progress, places=2)
        self._assert_zero_writes()

        # Abort mid-execution is covered by pause_resume_abort; Succeeded only
        # permits Plan (not Abort) per DeploymentStateMachine permissions.


if __name__ == "__main__":
    rospy.init_node("test_synchronized_dry_run")
    rostest.rosrun("remani_real", "test_synchronized_dry_run", SynchronizedDryRunTest)
# ################################
# Python: synchronized dry-run rostest end
# ################################
