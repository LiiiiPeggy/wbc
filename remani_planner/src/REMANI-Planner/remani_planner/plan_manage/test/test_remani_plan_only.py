#!/usr/bin/env python3

import copy
import threading
import unittest

import rospy
import rostest
from quadrotor_msgs.msg import PolynomialTraj
from remani_real_msgs.msg import ExecutionState, FrozenCandidate, PlannerStatus
from std_msgs.msg import Bool


class RemaniPlanOnlyTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._final = threading.Event()
        self._idle_after_handoff = threading.Event()
        self._actions = []
        self._adds = []
        self._statuses = []
        self._finish_count = 0
        self._saw_handoff = False
        self._stale_acknowledgements_scheduled = False
        self._stale_invalid_sent = threading.Event()
        self._candidate_duration = 0.0
        self._frozen_candidate = None
        self._raw_transaction_stamp = None
        self._frozen_pub = rospy.Publisher(
            "/remani/frozen_candidate", FrozenCandidate, queue_size=1)
        self._execution_state_pub = rospy.Publisher(
            "/remani/execution_state", ExecutionState, queue_size=1)
        self._candidate_sub = rospy.Subscriber(
            "/remani/planner_candidate", PolynomialTraj,
            self._candidate_callback, queue_size=20)
        self._status_sub = rospy.Subscriber(
            "/remani/planner_status", PlannerStatus,
            self._status_callback, queue_size=20)
        self._finish_sub = rospy.Subscriber(
            "/planning/finish", Bool, self._finish_callback, queue_size=10)
        rospy.set_param("/test_remani_plan_only/early_goal_sent", False)
        rospy.set_param("/test_remani_plan_only/ready", True)

    def _candidate_callback(self, message):
        with self._lock:
            self._actions.append(message.action)
            if message.action == PolynomialTraj.ACTION_WARN_START:
                self._raw_transaction_stamp = message.header.stamp
            if message.action == PolynomialTraj.ACTION_ADD:
                self._adds.append(message)
                self._candidate_duration += sum(
                    piece.duration for piece in message.trajectory)
            if message.action != PolynomialTraj.ACTION_WARN_FINAL:
                return
            self._final.set()
            if self._stale_acknowledgements_scheduled:
                return
            frozen = FrozenCandidate()
            frozen.header.stamp = rospy.Time.now()
            frozen.candidate_id = 42
            frozen.raw_transaction_stamp = self._raw_transaction_stamp
            frozen.complete = True
            frozen.valid = True
            frozen.duration = self._candidate_duration
            frozen.segments = list(self._adds)
            frozen.validation_code = "OK"
            frozen.validation_message = "accepted by fake Gate"
            self._frozen_candidate = frozen
            self._maybe_ack_locked()

    def _status_callback(self, message):
        with self._lock:
            self._statuses.append(message)
            if message.state == PlannerStatus.HANDOFF:
                self._saw_handoff = True
                self._maybe_ack_locked()
            elif message.state == PlannerStatus.IDLE and self._saw_handoff:
                self._idle_after_handoff.set()

    def _maybe_ack_locked(self):
        if (not self._saw_handoff or self._frozen_candidate is None or
                self._stale_acknowledgements_scheduled):
            return
        self._stale_acknowledgements_scheduled = True
        frozen = self._frozen_candidate
        stale_success = copy.deepcopy(frozen)
        stale_success.candidate_id = 41
        stale_success.raw_transaction_stamp -= rospy.Duration(1.0)
        rospy.Timer(rospy.Duration(0.05),
                    lambda _event: self._frozen_pub.publish(stale_success),
                    oneshot=True)

        def publish_stale_invalid(_event):
            invalid = ExecutionState()
            invalid.transaction_state = ExecutionState.TRANSACTION_INVALID
            invalid.candidate_id = 41
            invalid.raw_transaction_stamp = stale_success.raw_transaction_stamp
            invalid.last_error_code = "STALE_GATE_FAILURE"
            self._execution_state_pub.publish(invalid)
            self._stale_invalid_sent.set()

        rospy.Timer(rospy.Duration(0.10), publish_stale_invalid, oneshot=True)

    def _publish_matching_acknowledgement(self):
        with self._lock:
            frozen = self._frozen_candidate
            raw_transaction_stamp = self._raw_transaction_stamp
        self.assertIsNotNone(frozen, "fake Gate has no matching candidate")
        self.assertEqual(raw_transaction_stamp, frozen.raw_transaction_stamp)
        self._frozen_pub.publish(frozen)

    def _finish_callback(self, _message):
        with self._lock:
            self._finish_count += 1

    def test_real_planner_hands_off_without_internal_execution(self):
        early_goal_deadline = rospy.Time.now() + rospy.Duration(10.0)
        while (not rospy.get_param("/test_remani_plan_only/early_goal_sent",
                                   False) and
               rospy.Time.now() < early_goal_deadline):
            rospy.sleep(0.01)
        self.assertTrue(
            rospy.get_param("/test_remani_plan_only/early_goal_sent", False),
            "fake state did not publish the early startup target")
        rospy.sleep(0.2)
        with self._lock:
            self.assertNotIn(
                PlannerStatus.PLANNING,
                [status.state for status in self._statuses],
                "planner accepted the early target before reaching WAIT_TARGET")

        self.assertTrue(self._final.wait(45.0),
                        "planner did not publish a complete raw transaction")
        self.assertTrue(self._stale_invalid_sent.wait(5.0),
                        "fake Gate did not publish the stale invalid acknowledgement")
        rospy.sleep(0.50)
        with self._lock:
            self.assertEqual(PlannerStatus.HANDOFF, self._statuses[-1].state,
                             "planner accepted a stale Gate acknowledgement")
            self.assertFalse(
                self._idle_after_handoff.is_set(),
                "planner returned IDLE after a stale Gate acknowledgement")
        self._publish_matching_acknowledgement()
        self.assertTrue(self._idle_after_handoff.wait(10.0),
                        "planner did not return IDLE after Gate acknowledgement")

        with self._lock:
            actions_at_handoff = list(self._actions)
            status_states = [status.state for status in self._statuses]
            finish_count = self._finish_count
            duration = self._candidate_duration

        self.assertGreater(duration, 0.0)
        self.assertEqual(
            [PolynomialTraj.ACTION_WARN_START,
             PolynomialTraj.ACTION_ADD,
             PolynomialTraj.ACTION_WARN_FINAL],
            actions_at_handoff)
        self.assertEqual(0, finish_count)
        planning_index = status_states.index(PlannerStatus.PLANNING)
        handoff_index = status_states.index(PlannerStatus.HANDOFF,
                                            planning_index + 1)
        idle_index = status_states.index(PlannerStatus.IDLE, handoff_index + 1)
        self.assertLess(planning_index, handoff_index)
        self.assertLess(handoff_index, idle_index)
        self.assertEqual(PlannerStatus.IDLE, status_states[-1])

        rospy.sleep(duration + 1.0)
        with self._lock:
            self.assertEqual(actions_at_handoff, self._actions)
            self.assertEqual(0, self._finish_count)
            self.assertEqual(PlannerStatus.IDLE, self._statuses[-1].state)


if __name__ == "__main__":
    rospy.init_node("test_remani_plan_only")
    rostest.rosrun("remani_planner", "test_remani_plan_only",
                   RemaniPlanOnlyTest)
