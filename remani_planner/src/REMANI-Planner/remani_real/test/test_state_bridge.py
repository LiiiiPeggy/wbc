#!/usr/bin/env python3
# ################################
# Python: State Bridge rostest begin
# ################################
"""Rostest: remani_state_bridge_node mapping, TF, and fail-closed duplicate reject."""

from __future__ import print_function

import threading
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from remani_real_msgs.msg import Cr10Status
from sensor_msgs.msg import JointState


class StateBridgeTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._planning = None
        self._robot_model = None
        self._planning_seq = 0
        self._robot_model_seq = 0
        self._raw_pub = rospy.Publisher(
            "/remani/cr10_joint_states_raw", JointState, queue_size=1)
        self._odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self._status_pub = rospy.Publisher(
            "/remani/cr10_status", Cr10Status, queue_size=1)
        self._planning_sub = rospy.Subscriber(
            "/remani/cr10_joint_states", JointState, self._on_planning, queue_size=10)
        self._robot_model_sub = rospy.Subscriber(
            "/joint_states", JointState, self._on_robot_model, queue_size=10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.sleep(1.0)

    def _on_planning(self, msg):
        with self._lock:
            self._planning = msg
            self._planning_seq += 1

    def _on_robot_model(self, msg):
        with self._lock:
            self._robot_model = msg
            self._robot_model_seq += 1

    def _publish_valid_raw(self, stamp, positions=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)):
        raw = JointState()
        raw.header.stamp = stamp
        # Shuffled order on the wire; mapper must reorder by name.
        raw.name = ["joint3", "joint1", "joint6", "joint2", "joint5", "joint4"]
        raw.position = [
            positions[2], positions[0], positions[5],
            positions[1], positions[4], positions[3],
        ]
        self._raw_pub.publish(raw)

    def _publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.1
        odom.pose.pose.position.y = -0.2
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self._odom_pub.publish(odom)

    def _publish_status(self, stamp):
        status = Cr10Status()
        status.header.stamp = stamp
        status.connected = True
        status.enabled = True
        status.error_status = 0
        status.robot_mode = 5
        self._status_pub.publish(status)

    def _wait_outputs(self, min_planning_seq, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            with self._lock:
                if (self._planning is not None and
                        self._robot_model is not None and
                        self._planning_seq >= min_planning_seq):
                    return self._planning, self._robot_model, self._planning_seq
            rospy.sleep(0.05)
        with self._lock:
            return self._planning, self._robot_model, self._planning_seq

    def test_maps_planning_robot_model_and_tf(self):
        stamp = rospy.Time.now()
        self._publish_status(stamp)
        self._publish_odom(stamp)
        self._publish_valid_raw(stamp)

        planning, robot_model, seq = self._wait_outputs(1)
        self.assertIsNotNone(planning)
        self.assertIsNotNone(robot_model)
        self.assertEqual(6, len(planning.position))
        self.assertEqual(
            ["cr10_joint1", "cr10_joint2", "cr10_joint3",
             "cr10_joint4", "cr10_joint5", "cr10_joint6"],
            list(planning.name))
        self.assertEqual(
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            list(planning.position))
        self.assertGreater(len(robot_model.name), 6)
        self.assertIn("gripper_finger1_joint", list(robot_model.name))
        self.assertEqual(6, len(planning.name))
        # Until velocity estimate is valid, planning velocity must stay empty.
        self.assertEqual(0, len(planning.velocity))

        transform = None
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < deadline and transform is None:
            try:
                transform = self._tf_buffer.lookup_transform(
                    "world", "base_link", rospy.Time(0), rospy.Duration(0.2))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.sleep(0.05)
        self.assertIsNotNone(transform)
        self.assertEqual("world", transform.header.frame_id)
        self.assertEqual("base_link", transform.child_frame_id)
        self.assertAlmostEqual(0.1, transform.transform.translation.x, places=5)
        self.assertAlmostEqual(-0.2, transform.transform.translation.y, places=5)

        # Duplicate joint1 must not advance either output.
        with self._lock:
            before_planning = self._planning_seq
            before_robot = self._robot_model_seq
        bad = JointState()
        bad.header.stamp = rospy.Time.now()
        bad.name = ["joint1", "joint1", "joint2", "joint3", "joint4", "joint5"]
        bad.position = [9.0, 8.0, 2.0, 3.0, 4.0, 5.0]
        self._raw_pub.publish(bad)
        rospy.sleep(0.5)
        with self._lock:
            self.assertEqual(before_planning, self._planning_seq)
            self.assertEqual(before_robot, self._robot_model_seq)


if __name__ == "__main__":
    rospy.init_node("test_state_bridge")
    rostest.rosrun("remani_real", "test_state_bridge", StateBridgeTest)
# ################################
# Python: State Bridge rostest end
# ################################
