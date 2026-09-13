#!/usr/bin/env python3
# ################################
# Python: ranger-only staged execution rostest begin
# ################################
from __future__ import print_function

import subprocess
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from remani_real_msgs.msg import ExecutionState
from remani_real_msgs.srv import ExecuteCandidate
from std_msgs.msg import Bool, Empty, String, UInt32
from std_srvs.srv import Trigger


HARDWARE_TOPIC = "/remani/hardware/ranger/cmd_vel"


class RangerOnlyExecutionTest(unittest.TestCase):
    _TIMEOUT = 30.0

    def setUp(self):
        self._lock = threading.Lock()
        self._state = None
        self._cmds = []
        self._cmd_stamps = []
        self._timed_out = False
        self._goal_count = 0

        rospy.Subscriber("/remani/execution_state", ExecutionState, self._on_state, queue_size=10)
        rospy.Subscriber(HARDWARE_TOPIC, Twist, self._on_cmd, queue_size=20)
        rospy.Subscriber("/remani/ranger_watchdog_timed_out", Bool, self._on_timeout, queue_size=1)
        rospy.Subscriber("/remani/test/cr10_action_goal_count", UInt32, self._on_goal, queue_size=1)
        self._plan_pub = rospy.Publisher("/ee_goal_plan", Empty, queue_size=1)
        self._scenario_pub = rospy.Publisher(
            "/remani/test/planner_scenario", String, queue_size=1, latch=True)
        self._reset_pub = rospy.Publisher(
            "/remani/test/reset_ranger", Empty, queue_size=1)

        rospy.sleep(1.0)
        self._scenario_pub.publish(String(data="happy"))
        self._recover()

    def _on_state(self, msg):
        with self._lock:
            self._state = msg

    def _on_cmd(self, msg):
        with self._lock:
            self._cmds.append(msg)
            self._cmd_stamps.append(rospy.Time.now().to_sec())

    def _on_timeout(self, msg):
        with self._lock:
            self._timed_out = bool(msg.data)

    def _on_goal(self, msg):
        with self._lock:
            self._goal_count = msg.data

    def _latest(self):
        with self._lock:
            return self._state

    def _wait(self, pred, desc):
        deadline = rospy.Time.now() + rospy.Duration(self._TIMEOUT)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            msg = self._latest()
            if msg is not None and pred(msg):
                return msg
            rospy.sleep(0.05)
        msg = self._latest()
        self.fail("timeout waiting for {0}; last_state={1}".format(desc, msg))

    def _abort(self):
        abort = rospy.ServiceProxy("/remani/abort", Trigger)
        abort.wait_for_service(timeout=self._TIMEOUT)
        return abort()

    def _pause(self):
        pause = rospy.ServiceProxy("/remani/pause", Trigger)
        pause.wait_for_service(timeout=self._TIMEOUT)
        return pause()


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

    def _recover(self):
        msg = self._latest()
        if msg is not None and msg.executor_state != ExecutionState.EXECUTOR_NONE:
            self._abort()
        self._wait(
            lambda m: m.executor_state in (
                ExecutionState.EXECUTOR_NONE, ExecutionState.EXECUTOR_SUCCEEDED)
            and RangerOnlyExecutionTest._readiness_ok(m),
            "ready-ish with full readiness")

    def _plan_and_execute(self):
        with self._lock:
            self._cmds = []
            self._cmd_stamps = []
        self._reset_pub.publish(Empty())
        rospy.sleep(0.2)
        self._plan_pub.publish(Empty())
        planned = self._wait(
            lambda m: m.candidate_valid and m.executor_state == ExecutionState.EXECUTOR_PLANNED,
            "PLANNED")
        execute = rospy.ServiceProxy("/remani/execute", ExecuteCandidate)
        execute.wait_for_service(timeout=self._TIMEOUT)
        accepted = execute(planned.candidate_id)
        self.assertTrue(accepted.accepted)
        return planned.candidate_id

    def test_ranger_only_watchdog_abort_and_rogue(self):
        self._plan_and_execute()
        # Wait until hardware commands appear.
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < deadline:
            with self._lock:
                if self._cmds:
                    break
            rospy.sleep(0.05)
        with self._lock:
            self.assertGreater(len(self._cmds), 0)
            max_abs_linear = max(abs(c.linear.x) for c in self._cmds)
        self.assertLessEqual(max_abs_linear, 0.05)

        # Pause stops Executor publishes; fake driver watchdog should zero.
        paused = self._pause()
        self.assertTrue(paused.success)
        pause_t = time.time()
        watchdog_stop_latency = None
        while time.time() - pause_t < 1.0:
            with self._lock:
                if self._timed_out:
                    watchdog_stop_latency = time.time() - pause_t
                    break
            time.sleep(0.02)
        self.assertIsNotNone(watchdog_stop_latency)
        self.assertLessEqual(watchdog_stop_latency, 0.25)

        abort_resp = self._abort()
        self.assertTrue(
            abort_resp.success,
            "abort failed: {0} state={1}".format(
                abort_resp.message, self._latest()))
        self._wait(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_NONE,
            "READY after abort")

        # Second execute then Abort while moving.
        with self._lock:
            before = len(self._cmds)
        self._plan_and_execute()
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < deadline:
            with self._lock:
                if len(self._cmds) > before:
                    break
            rospy.sleep(0.05)
        abort_resp = self._abort()
        self.assertTrue(abort_resp.success, abort_resp.message)
        rospy.sleep(0.3)
        with self._lock:
            after_abort = self._cmds[before:]
        self.assertGreater(len(after_abort), 0, "expected hardware cmds before/during abort")
        # Abort publishes an explicit zero as the recovery command.
        self.assertTrue(
            any(abs(c.linear.x) <= 1e-9 and abs(c.angular.z) <= 1e-9
                for c in after_abort),
            after_abort[-5:])
        self._wait(
            lambda m: m.executor_state == ExecutionState.EXECUTOR_NONE,
            "READY after second abort")

        with self._lock:
            self.assertEqual(0, self._goal_count)

        # Rogue publisher -> ownership ERROR.
        self._plan_and_execute()
        rospy.sleep(0.2)
        rogue = subprocess.Popen(
            [
                "rostopic", "pub", "-r", "20", HARDWARE_TOPIC,
                "geometry_msgs/Twist", "{linear: {x: 0.01}}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            self._wait(
                lambda m: m.executor_state == ExecutionState.EXECUTOR_ERROR,
                "ERROR on rogue publisher")
        finally:
            rogue.terminate()
            try:
                rogue.wait(timeout=2)
            except Exception:
                rogue.kill()


if __name__ == "__main__":
    rospy.init_node("test_ranger_only_execution")
    rostest.rosrun("remani_real", "test_ranger_only_execution", RangerOnlyExecutionTest)
# ################################
# Python: ranger-only staged execution rostest end
# ################################
