#!/usr/bin/env python3
# ################################
# Python: fake CR10 FollowJointTrajectory Action server begin
# ################################
"""Contract-faithful fake CR10 Action server for staged CR10-only rostest."""

from __future__ import print_function

import threading
import time

import actionlib
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from remani_real_msgs.srv import (
    Cr10EmergencyStop,
    Cr10EmergencyStopResponse,
    Cr10Stop,
    Cr10StopResponse,
)
from std_msgs.msg import Float64, UInt32
from trajectory_msgs.msg import JointTrajectoryPoint


CANONICAL = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class FakeCr10ActionServer(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._goal_count = 0
        self._cancel_count = 0
        self._stop_count = 0
        self._servoj_samples = 0
        self._non_hold_before_hold_end = 0
        self._active_gh = None
        self._cancel_requested = False
        self._q = [0.0] * 6
        self._qd = [0.0] * 6
        self._t0 = None
        self._traj = None
        self._hold_mode = rospy.get_param("~remani_prestart_hold_mode", True)
        self._servoj_period = float(rospy.get_param("~servoj_period", 0.05))
        self._goal_tol = float(rospy.get_param("~cr10_goal_joint_tol", 0.02))
        self._stop_tol = float(rospy.get_param("~cr10_stop_velocity_tol", 0.01))
        self._settle_timeout = float(
            rospy.get_param("~completion_settle_timeout", 2.0))
        self._required_settle = int(
            rospy.get_param("~required_settle_samples", 3))

        self._goal_pub = rospy.Publisher(
            "/remani/test/cr10_action_goal_count", UInt32, queue_size=1, latch=True)
        self._cancel_pub = rospy.Publisher(
            "/remani/test/cr10_action_cancel_count", UInt32, queue_size=1, latch=True)
        self._stop_pub = rospy.Publisher(
            "/remani/test/cr10_stop_count", UInt32, queue_size=1, latch=True)
        self._sample_pub = rospy.Publisher(
            "/remani/test/cr10_servoj_sample_count", UInt32, queue_size=1, latch=True)
        self._before_hold_pub = rospy.Publisher(
            "/remani/test/cr10_servoj_before_hold_end", UInt32, queue_size=1, latch=True)
        self._first_non_hold_pub = rospy.Publisher(
            "/remani/cr10_first_non_hold_servoj_steady", Float64, queue_size=1, latch=True)

        self._as = actionlib.ActionServer(
            "/cr10_robot/joint_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            goal_cb=self._on_goal,
            cancel_cb=self._on_cancel,
            auto_start=False)
        self._as.start()

        self._stop_srv = rospy.Service(
            "/dobot_v4_bringup/srv/Stop", Cr10Stop, self._on_stop)
        self._estop_srv = rospy.Service(
            "/dobot_v4_bringup/srv/EmergencyStop", Cr10EmergencyStop, self._on_estop)

        self._publish_counters()
        self._timer = rospy.Timer(
            rospy.Duration(self._servoj_period), self._on_tick)
        rospy.loginfo("fake_cr10_action_server online")

    def _publish_counters(self):
        self._goal_pub.publish(UInt32(data=self._goal_count))
        self._cancel_pub.publish(UInt32(data=self._cancel_count))
        self._stop_pub.publish(UInt32(data=self._stop_count))
        self._sample_pub.publish(UInt32(data=self._servoj_samples))
        self._before_hold_pub.publish(UInt32(data=self._non_hold_before_hold_end))

    def _validate(self, trajectory):
        if len(trajectory.joint_names) != 6:
            return "NAME_COUNT"
        if len(set(trajectory.joint_names)) != 6:
            return "NAME_DUPLICATE"
        if set(trajectory.joint_names) != set(CANONICAL):
            return "NAME_SET"
        if len(trajectory.points) < 2:
            return "POINT_COUNT"
        prev_t = None
        for point in trajectory.points:
            if len(point.positions) != 6:
                return "POSITION_SIZE"
            if len(point.velocities) != 6:
                return "VELOCITY_SIZE"
            for v in list(point.positions) + list(point.velocities):
                if v != v:
                    return "POSITION_NONFINITE"
            t = point.time_from_start.to_sec()
            if prev_t is not None and not (t > prev_t):
                return "TIME_NOT_STRICT"
            prev_t = t
        if self._hold_mode:
            if len(trajectory.points) < 3:
                return "REMANI_HOLD_POINTS"
            if abs(trajectory.points[0].time_from_start.to_sec()) > 1e-12:
                return "REMANI_HOLD_POINT0"
            if max(abs(v) for v in trajectory.points[0].velocities) > 1e-12:
                return "REMANI_HOLD_POINT0"
            if not (trajectory.points[1].time_from_start.to_sec() > 0.0):
                return "REMANI_HOLD_LEAD"
        return None

    def _reorder(self, trajectory):
        index = {name: i for i, name in enumerate(trajectory.joint_names)}
        order = [index[n] for n in CANONICAL]
        trajectory.joint_names = list(CANONICAL)
        new_points = []
        for point in trajectory.points:
            p = JointTrajectoryPoint()
            p.time_from_start = point.time_from_start
            p.positions = [point.positions[i] for i in order]
            p.velocities = [point.velocities[i] for i in order]
            new_points.append(p)
        trajectory.points = new_points
        return trajectory

    def _sample_at(self, elapsed):
        points = self._traj.points
        hold_until = points[1].time_from_start.to_sec() if self._hold_mode else 0.0
        if self._hold_mode and elapsed + 1e-12 < hold_until:
            return list(points[0].positions), list(points[0].velocities), True
        if elapsed <= points[0].time_from_start.to_sec():
            return list(points[0].positions), list(points[0].velocities), False
        if elapsed >= points[-1].time_from_start.to_sec():
            return list(points[-1].positions), [0.0] * 6, False
        for i in range(len(points) - 1):
            t0 = points[i].time_from_start.to_sec()
            t1 = points[i + 1].time_from_start.to_sec()
            if t0 <= elapsed <= t1:
                alpha = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
                q = [
                    points[i].positions[j] + alpha * (
                        points[i + 1].positions[j] - points[i].positions[j])
                    for j in range(6)
                ]
                qd = [
                    points[i].velocities[j] + alpha * (
                        points[i + 1].velocities[j] - points[i].velocities[j])
                    for j in range(6)
                ]
                return q, qd, False
        return list(points[-1].positions), [0.0] * 6, False

    def _on_goal(self, gh):
        with self._lock:
            if self._active_gh is not None:
                result = FollowJointTrajectoryResult()
                result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                gh.set_rejected(result, "GOAL_ACTIVE")
                return
            err = self._validate(gh.get_goal().trajectory)
            if err is not None:
                result = FollowJointTrajectoryResult()
                result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                gh.set_rejected(result, err)
                self._publish_counters()
                return
            self._traj = self._reorder(gh.get_goal().trajectory)
            self._t0 = time.time()
            self._cancel_requested = False
            self._servoj_samples = 0
            self._non_hold_before_hold_end = 0
            self._goal_count += 1
            self._active_gh = gh
            self._q = list(self._traj.points[0].positions)
            self._qd = [0.0] * 6
            self._settle_good = 0
            self._cancel_good = 0
            self._settle_start = None
            self._published_first_non_hold = False
            gh.set_accepted()
            self._publish_counters()

    def _on_cancel(self, gh):
        with self._lock:
            self._cancel_count += 1
            self._cancel_requested = True
            self._qd = [0.0] * 6
            self._publish_counters()

    def _on_stop(self, _req):
        with self._lock:
            self._stop_count += 1
            self._cancel_requested = True
            self._qd = [0.0] * 6
            self._publish_counters()
        return Cr10StopResponse(res=0)

    def _on_estop(self, _req):
        with self._lock:
            self._stop_count += 1
            self._cancel_requested = True
            self._qd = [0.0] * 6
            self._publish_counters()
        return Cr10EmergencyStopResponse(res=0)

    def _terminal(self, status_text, canceled=False, aborted=False, succeeded=False):
        result = FollowJointTrajectoryResult()
        gh = self._active_gh
        self._active_gh = None
        self._traj = None
        if gh is None:
            return
        if succeeded:
            gh.set_succeeded(result, status_text)
        elif canceled:
            gh.set_canceled(result, status_text)
        else:
            result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            gh.set_aborted(result, status_text)

    def _on_tick(self, _event):
        with self._lock:
            if self._active_gh is None or self._traj is None or self._t0 is None:
                return
            now = time.time()
            elapsed = now - self._t0
            force_settle_fail = rospy.get_param("~force_settle_fail", False)

            if self._cancel_requested:
                self._cancel_good += 1
                if self._cancel_good >= self._required_settle:
                    self._terminal("stopped", canceled=True)
                self._publish_counters()
                return

            cmd_q, cmd_qd, in_hold = self._sample_at(elapsed)
            self._servoj_samples += 1
            if in_hold:
                # Hold ServoJ only; any non-hold command here would increment the gate.
                pass
            else:
                hold_until = (
                    self._traj.points[1].time_from_start.to_sec()
                    if self._hold_mode else 0.0)
                if self._hold_mode and elapsed + 1e-12 < hold_until:
                    self._non_hold_before_hold_end += 1
                if not self._published_first_non_hold:
                    self._published_first_non_hold = True
                    self._first_non_hold_pub.publish(Float64(data=now))

            self._q = cmd_q
            self._qd = cmd_qd
            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = list(CANONICAL)
            feedback.actual.positions = list(self._q)
            feedback.actual.velocities = list(self._qd)
            feedback.desired.positions = list(cmd_q)
            self._active_gh.publish_feedback(feedback)

            t_end = self._traj.points[-1].time_from_start.to_sec()
            if elapsed + 1e-12 >= t_end:
                if self._settle_start is None:
                    self._settle_start = now
                    self._settle_good = 0
                goal_q = list(self._traj.points[-1].positions)
                within = all(
                    abs(self._q[i] - goal_q[i]) <= self._goal_tol for i in range(6))
                stopped = all(abs(self._qd[i]) <= self._stop_tol for i in range(6))
                if force_settle_fail:
                    within = False
                if within and stopped:
                    self._settle_good += 1
                else:
                    self._settle_good = 0
                if self._settle_good >= self._required_settle:
                    self._terminal("settled", succeeded=True)
                elif (now - self._settle_start) > self._settle_timeout:
                    self._terminal("completion tolerance", aborted=True)
            self._publish_counters()


def main():
    rospy.init_node("fake_cr10_action_server")
    FakeCr10ActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
# ################################
# Python: fake CR10 FollowJointTrajectory Action server end
# ################################
