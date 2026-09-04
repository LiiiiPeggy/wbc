#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from quadrotor_msgs.msg import PolynomialTraj
from remani_real_msgs.msg import FrozenCandidate, PlannerStatus
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
        self._ack_sent = False
        self._candidate_duration = 0.0
        self._frozen_candidate = None
        self._frozen_pub = rospy.Publisher(
            "/remani/frozen_candidate", FrozenCandidate, queue_size=1)
        self._candidate_sub = rospy.Subscriber(
            "/remani/planner_candidate", PolynomialTraj,
            self._candidate_callback, queue_size=20)
        self._status_sub = rospy.Subscriber(
            "/remani/planner_status", PlannerStatus,
            self._status_callback, queue_size=20)
        self._finish_sub = rospy.Subscriber(
            "/planning/finish", Bool, self._finish_callback, queue_size=10)
        rospy.set_param("/test_remani_plan_only/ready", True)

    def _candidate_callback(self, message):
        with self._lock:
            self._actions.append(message.action)
            if message.action == PolynomialTraj.ACTION_ADD:
                self._adds.append(message)
                self._candidate_duration += sum(
                    piece.duration for piece in message.trajectory)
            if message.action != PolynomialTraj.ACTION_WARN_FINAL:
                return
            self._final.set()
            if self._ack_sent:
                return
            frozen = FrozenCandidate()
            frozen.header.stamp = rospy.Time.now()
            frozen.candidate_id = 42
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
                self._ack_sent):
            return
        self._ack_sent = True
        frozen = self._frozen_candidate
        rospy.Timer(rospy.Duration(0.1),
                    lambda _event: self._frozen_pub.publish(frozen),
                    oneshot=True)

    def _finish_callback(self, _message):
        with self._lock:
            self._finish_count += 1

    def test_real_planner_hands_off_without_internal_execution(self):
        self.assertTrue(self._final.wait(45.0),
                        "planner did not publish a complete raw transaction")
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
