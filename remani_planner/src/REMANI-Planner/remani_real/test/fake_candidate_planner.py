#!/usr/bin/env python3
# ################################
# Python: fake candidate planner begin
# ################################
"""Deterministic fake planner for dry-run control-plane rostest."""

from __future__ import print_function

import threading

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PolynomialMatrix, PolynomialTraj
from remani_real_msgs.msg import PlannerStatus
from std_msgs.msg import String


class FakeCandidatePlanner(object):
    _MSG_GAP = 0.05

    def __init__(self):
        self._lock = threading.Lock()
        self._scenario = "happy"
        self._status_pub = rospy.Publisher(
            "/remani/planner_status", PlannerStatus, queue_size=1)
        self._candidate_pub = rospy.Publisher(
            "/remani/planner_candidate", PolynomialTraj, queue_size=10)
        rospy.Subscriber("/ee_goal", PoseStamped, self._on_ee_goal, queue_size=1)
        rospy.Subscriber(
            "/remani/test/planner_scenario", String, self._on_scenario, queue_size=1)

    def _on_scenario(self, msg):
        scenario = (msg.data or "happy").strip().lower()
        with self._lock:
            self._scenario = scenario
        rospy.loginfo("Fake planner scenario set to '%s'", scenario)

    def _current_scenario(self):
        with self._lock:
            return self._scenario

    def _publish_status(self, state):
        msg = PlannerStatus()
        msg.header.stamp = rospy.Time.now()
        msg.state = state
        self._status_pub.publish(msg)

    def _control_message(self, action, trajectory_id=0, singul=0, trajectory=None):
        msg = PolynomialTraj()
        msg.header.stamp = rospy.Time.now()
        msg.action = action
        msg.trajectory_id = trajectory_id
        msg.singul = singul
        if trajectory is not None:
            msg.trajectory = trajectory
        return msg

    def _valid_add(self, trajectory_id):
        piece = PolynomialMatrix()
        piece.num_dim = 8
        piece.num_order = 7
        piece.duration = 1.0
        piece.data = [0.0] * 64
        piece.data[56] = 0.0
        piece.data[48] = 0.05
        for idx in range(57, 64):
            piece.data[idx] = 0.0
        return self._control_message(
            PolynomialTraj.ACTION_ADD,
            trajectory_id=trajectory_id,
            singul=1,
            trajectory=[piece])

    def _emit_happy(self):
        self._candidate_pub.publish(
            self._control_message(PolynomialTraj.ACTION_WARN_START))
        rospy.sleep(self._MSG_GAP)
        self._candidate_pub.publish(self._valid_add(1))
        rospy.sleep(self._MSG_GAP)
        self._candidate_pub.publish(
            self._control_message(PolynomialTraj.ACTION_WARN_FINAL))
        rospy.sleep(self._MSG_GAP)
        self._publish_status(PlannerStatus.HANDOFF)
        rospy.sleep(self._MSG_GAP)
        self._publish_status(PlannerStatus.IDLE)

    def _emit_add_before_start(self):
        self._candidate_pub.publish(self._valid_add(1))
        rospy.sleep(self._MSG_GAP)
        self._publish_status(PlannerStatus.IDLE)

    def _emit_duplicate_add(self):
        self._candidate_pub.publish(
            self._control_message(PolynomialTraj.ACTION_WARN_START))
        rospy.sleep(self._MSG_GAP)
        self._candidate_pub.publish(self._valid_add(1))
        rospy.sleep(self._MSG_GAP)
        self._candidate_pub.publish(self._valid_add(1))
        rospy.sleep(self._MSG_GAP)
        self._publish_status(PlannerStatus.IDLE)

    def _emit_skipped_id(self):
        self._candidate_pub.publish(
            self._control_message(PolynomialTraj.ACTION_WARN_START))
        rospy.sleep(self._MSG_GAP)
        self._candidate_pub.publish(self._valid_add(2))
        rospy.sleep(self._MSG_GAP)
        self._publish_status(PlannerStatus.IDLE)

    def _emit_hang_after_add(self):
        self._candidate_pub.publish(
            self._control_message(PolynomialTraj.ACTION_WARN_START))
        rospy.sleep(self._MSG_GAP)
        self._candidate_pub.publish(self._valid_add(1))
        rospy.sleep(self._MSG_GAP)
        # Intentionally omit FINAL so assembly_timeout can fire.

    def _on_ee_goal(self, _msg):
        scenario = self._current_scenario()
        rospy.loginfo(
            "Fake planner received /ee_goal; scenario='%s'", scenario)
        self._publish_status(PlannerStatus.PLANNING)
        rospy.sleep(self._MSG_GAP)

        if scenario == "add_before_start":
            self._emit_add_before_start()
        elif scenario == "duplicate_add":
            self._emit_duplicate_add()
        elif scenario == "skipped_id":
            self._emit_skipped_id()
        elif scenario == "hang_after_add":
            self._emit_hang_after_add()
        elif scenario == "idle":
            self._publish_status(PlannerStatus.IDLE)
        else:
            self._emit_happy()


def main():
    rospy.init_node("fake_candidate_planner")
    FakeCandidatePlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
# ################################
# Python: fake candidate planner end
# ################################
