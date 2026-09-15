#!/usr/bin/env python3
# ################################
# Python: real launch contract rostest begin
# ################################
from __future__ import print_function

import threading
import unittest

import rosgraph
import rospy
import rostest
import tf2_ros
from remani_real_msgs.msg import ExecutionState
from sensor_msgs.msg import JointState


CANONICAL_HW = "/remani/hardware/ranger/cmd_vel"
STALE_HW = "/remani/ranger_cmd_vel_hw"
PLANNING_JOINTS = [
    "cr10_joint1",
    "cr10_joint2",
    "cr10_joint3",
    "cr10_joint4",
    "cr10_joint5",
    "cr10_joint6",
]


def publishers(topic):
    published, _, _ = rosgraph.Master("/real_launch_contract").getSystemState()
    for name, nodes in published:
        if name == topic:
            return nodes
    return []


def subscribers(topic):
    _, subscribed, _ = rosgraph.Master("/real_launch_contract").getSystemState()
    for name, nodes in subscribed:
        if name == topic:
            return nodes
    return []


class RealLaunchContractTest(unittest.TestCase):
    _TIMEOUT = 30.0

    def setUp(self):
        self._lock = threading.Lock()
        self._execution_state = None
        self._joint_state = None
        rospy.Subscriber(
            "/remani/execution_state", ExecutionState, self._on_state, queue_size=10)
        rospy.Subscriber(
            "/remani/cr10_joint_states", JointState, self._on_joints, queue_size=10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._wait_ready()

    def _on_state(self, msg):
        with self._lock:
            self._execution_state = msg

    def _on_joints(self, msg):
        with self._lock:
            self._joint_state = msg

    def _wait_ready(self):
        deadline = rospy.Time.now() + rospy.Duration(self._TIMEOUT)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            with self._lock:
                state = self._execution_state
                joints = self._joint_state
            if (
                state is not None
                and state.odom_ready
                and state.cr10_joint_ready
                and joints is not None
                and len(joints.name) >= 6
            ):
                return
            rospy.sleep(0.05)
        self.fail("Timed out waiting for launch readiness")

    def test_launch_contract(self):
        self.assertEqual(0, len(publishers(CANONICAL_HW)))
        self.assertEqual(0, len(publishers(STALE_HW)))

        bridge_nodes = subscribers("/remani/cr10_joint_states_raw")
        self.assertTrue(
            any("remani_state_bridge" in n for n in bridge_nodes),
            "State Bridge must subscribe to raw CR10 joints: {0}".format(bridge_nodes),
        )

        with self._lock:
            joints = self._joint_state
            state = self._execution_state
        self.assertEqual(PLANNING_JOINTS, list(joints.name[:6]))
        self.assertEqual(ExecutionState.ENV_STATIC_EMPTY, state.environment_mode)
        self.assertTrue(state.dry_run)

        tf_msg = None
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < deadline and tf_msg is None:
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    "world", "base_link", rospy.Time(0), rospy.Duration(0.2))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.sleep(0.05)
        self.assertIsNotNone(tf_msg)
        self.assertAlmostEqual(0.0, tf_msg.transform.translation.x, places=6)
        self.assertAlmostEqual(0.0, tf_msg.transform.translation.y, places=6)


if __name__ == "__main__":
    rospy.init_node("test_real_launch_contract")
    rostest.rosrun("remani_real", "test_real_launch_contract", RealLaunchContractTest)
# ################################
# Python: real launch contract rostest end
# ################################
