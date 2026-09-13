#!/usr/bin/env python3
# ################################
# Python: CR10-only staged execution rostest begin
# ################################
from __future__ import print_function

import threading
import time
import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import UInt32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ACTION_NAME = "/cr10_robot/joint_controller/follow_joint_trajectory"
CANONICAL = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _point(t, q, qd=None):
    p = JointTrajectoryPoint()
    p.time_from_start = rospy.Duration.from_sec(t)
    p.positions = list(q)
    p.velocities = list(qd if qd is not None else [0.0] * 6)
    return p


def hold_then_move(lead=1.0, dq=0.05, duration=1.0):
    traj = JointTrajectory()
    traj.joint_names = list(CANONICAL)
    q0 = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60]
    q1 = [x + dq for x in q0]
    traj.points = [
        _point(0.0, q0),
        _point(lead, q0),
        _point(lead + duration, q1),
    ]
    return traj


class Cr10OnlyExecutionTest(unittest.TestCase):
    _TIMEOUT = 30.0

    def setUp(self):
        self._lock = threading.Lock()
        self._before_hold = 0
        self._samples = 0
        rospy.Subscriber(
            "/remani/test/cr10_servoj_before_hold_end", UInt32, self._on_before,
            queue_size=1)
        rospy.Subscriber(
            "/remani/test/cr10_servoj_sample_count", UInt32, self._on_samples,
            queue_size=1)
        self._client = actionlib.SimpleActionClient(
            ACTION_NAME, FollowJointTrajectoryAction)
        self.assertTrue(self._client.wait_for_server(rospy.Duration(self._TIMEOUT)))
        rospy.set_param("/fake_cr10_action_server/force_settle_fail", False)
        rospy.sleep(0.3)

    def _on_before(self, msg):
        with self._lock:
            self._before_hold = msg.data

    def _on_samples(self, msg):
        with self._lock:
            self._samples = msg.data

    def _send(self, trajectory, timeout=20.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._client.send_goal(goal)
        finished = self._client.wait_for_result(rospy.Duration(timeout))
        self.assertTrue(finished)
        return self._client.get_state()

    def test_hold_cancel_settle_rejects_and_abort(self):
        # --- Hold gate: no non-hold ServoJ before lead ends ---
        traj = hold_then_move(lead=1.0, dq=0.05, duration=2.0)
        goal = FollowJointTrajectoryGoal(trajectory=traj)
        self._client.send_goal(goal)
        rospy.sleep(0.80)
        with self._lock:
            servo_samples_before_hold_end = self._before_hold
        self.assertEqual(0, servo_samples_before_hold_end)

        # --- Cancel mid-run: latency < 0.20 and PREEMPTED (not SUCCEEDED) ---
        t_cancel = time.time()
        self._client.cancel_goal()
        self.assertTrue(self._client.wait_for_result(rospy.Duration(5.0)))
        cancel_callback_latency = time.time() - t_cancel
        canceled_goal_status = self._client.get_state()
        self.assertLess(cancel_callback_latency, 0.20)
        self.assertEqual(GoalStatus.PREEMPTED, canceled_goal_status)
        self.assertNotEqual(GoalStatus.SUCCEEDED, canceled_goal_status)

        # --- Settled success ---
        rospy.set_param("/fake_cr10_action_server/force_settle_fail", False)
        settled_goal_status = self._send(
            hold_then_move(lead=0.4, dq=0.04, duration=0.4), timeout=15.0)
        self.assertEqual(GoalStatus.SUCCEEDED, settled_goal_status)

        # --- Tolerance / settle abort ---
        rospy.set_param("/fake_cr10_action_server/force_settle_fail", True)
        tolerance_failure_status = self._send(
            hold_then_move(lead=0.3, dq=0.03, duration=0.3), timeout=15.0)
        self.assertEqual(GoalStatus.ABORTED, tolerance_failure_status)
        rospy.set_param("/fake_cr10_action_server/force_settle_fail", False)

        # --- Malformed goals: REJECTED and zero additional ServoJ samples ---
        with self._lock:
            samples_before = self._samples

        bad_vel = hold_then_move()
        bad_vel.points[1].velocities = []
        self.assertEqual(GoalStatus.REJECTED, self._send(bad_vel, timeout=5.0))

        bad_time = hold_then_move()
        bad_time.points[-1].time_from_start = bad_time.points[-2].time_from_start
        self.assertEqual(GoalStatus.REJECTED, self._send(bad_time, timeout=5.0))

        bad_one = JointTrajectory()
        bad_one.joint_names = list(CANONICAL)
        bad_one.points = [_point(0.0, [0.0] * 6)]
        self.assertEqual(GoalStatus.REJECTED, self._send(bad_one, timeout=5.0))

        bad_dup = hold_then_move()
        bad_dup.joint_names[5] = "joint1"
        self.assertEqual(GoalStatus.REJECTED, self._send(bad_dup, timeout=5.0))

        bad_nan = hold_then_move()
        bad_nan.points[0].positions[0] = float("nan")
        self.assertEqual(GoalStatus.REJECTED, self._send(bad_nan, timeout=5.0))

        with self._lock:
            self.assertEqual(samples_before, self._samples)


if __name__ == "__main__":
    rospy.init_node("test_cr10_only_execution")
    rostest.rosrun("remani_real", "test_cr10_only_execution", Cr10OnlyExecutionTest)
# ################################
# Python: CR10-only staged execution rostest end
# ################################
