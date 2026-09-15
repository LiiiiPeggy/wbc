#!/usr/bin/env python3
# ################################
# Python: pause/resume/abort rostest begin
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
from std_srvs.srv import Trigger


def publishers(topic):
    published, _, _ = rosgraph.Master("/pause_resume_abort").getSystemState()
    for name, nodes in published:
        if name == topic:
            return nodes
    return []


class PauseResumeAbortTest(unittest.TestCase):
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
        self._wait_for(
            lambda m: m.odom_ready and m.cr10_joint_ready,
            self._TIMEOUT,
            "ready",
        )

    def _on_state(self, msg):
        with self._lock:
            self._state = msg

    def _on_counts(self, msg):
        with self._lock:
            self._counts = list(msg.data)

    def _latest(self):
        with self._lock:
            return self._state

    def _wait_for(self, predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            msg = self._latest()
            if msg is not None and predicate(msg):
                return msg
            rospy.sleep(0.05)
        self.fail("Timed out waiting for {0}".format(description))

    def _assert_zero_writes(self):
        self.assertEqual(0, len(publishers("/remani/hardware/ranger/cmd_vel")))
        with self._lock:
            counts = self._counts
        self.assertIsNotNone(counts)
        self.assertEqual([0, 0, 0, 0, 0], counts)

    def _plan_and_execute(self):
        self._plan_pub.publish(Empty())
        planned = self._wait_for(
            lambda m: m.candidate_valid
            and m.executor_state == ExecutionState.EXECUTOR_PLANNED
            and m.candidate_id > 0,
            self._TIMEOUT,
            "PLANNED",
        )
        execute = rospy.ServiceProxy("/remani/execute", ExecuteCandidate)
        execute.wait_for_service(timeout=self._TIMEOUT)
        self.assertTrue(execute(planned.candidate_id).accepted)
        self._wait_for(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_EXECUTING,
            self._TIMEOUT,
            "EXECUTING",
        )
        return planned.candidate_id

    def test_pause_resume_abort(self):
        self._plan_and_execute()
        # Wait until past T0 so pause has a non-zero param time.
        self._wait_for(
            lambda m: m.execution_progress > 0.05,
            self._TIMEOUT,
            "post-T0",
        )

        pause = rospy.ServiceProxy("/remani/pause", Trigger)
        pause.wait_for_service(timeout=self._TIMEOUT)
        self.assertTrue(pause().success)
        paused = self._wait_for(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_PAUSED,
            self._TIMEOUT,
            "PAUSED",
        )
        pause_param = paused.pause_param_time
        self.assertGreaterEqual(pause_param, 0.0)
        self._assert_zero_writes()

        resume = rospy.ServiceProxy("/remani/resume", Trigger)
        resume.wait_for_service(timeout=self._TIMEOUT)
        self.assertTrue(resume().success)
        self._wait_for(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_EXECUTING,
            self._TIMEOUT,
            "EXECUTING after resume",
        )
        self._assert_zero_writes()

        abort = rospy.ServiceProxy("/remani/abort", Trigger)
        abort.wait_for_service(timeout=self._TIMEOUT)
        self.assertTrue(abort().success)
        self._wait_for(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_NONE,
            self._TIMEOUT,
            "NONE after abort",
        )
        self._assert_zero_writes()


if __name__ == "__main__":
    rospy.init_node("test_pause_resume_abort")
    rostest.rosrun("remani_real", "test_pause_resume_abort", PauseResumeAbortTest)
# ################################
# Python: pause/resume/abort rostest end
# ################################
