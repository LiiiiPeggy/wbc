#!/usr/bin/env python3
# ################################
# Python: Ranger topic isolation rostest begin
# ################################
from __future__ import print_function

import subprocess
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist


HARDWARE_TOPIC = "/remani/hardware/ranger/cmd_vel"
ORDINARY_CMD = "/cmd_vel"


class FakeRangerSink(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.count = 0
        self.last = None
        self.sub = rospy.Subscriber(
            HARDWARE_TOPIC, Twist, self._cb, queue_size=10)

    def _cb(self, msg):
        with self.lock:
            self.count += 1
            self.last = msg


class RangerTopicIsolationTest(unittest.TestCase):
    def test_ordinary_cmd_vel_disconnected_and_ownership(self):
        sink = FakeRangerSink()
        rospy.sleep(0.5)

        ordinary_pub = rospy.Publisher(ORDINARY_CMD, Twist, queue_size=10)
        rospy.sleep(0.3)
        for _ in range(10):
            ordinary_pub.publish(Twist())
            rospy.sleep(0.05)
        with sink.lock:
            self.assertEqual(0, sink.count)

        hw_pub = rospy.Publisher(HARDWARE_TOPIC, Twist, queue_size=1, latch=False)
        rospy.sleep(0.5)
        msg = Twist()
        msg.linear.x = 0.05
        hw_pub.publish(msg)
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < deadline:
            with sink.lock:
                if sink.count >= 1:
                    break
            rospy.sleep(0.05)
        with sink.lock:
            self.assertGreaterEqual(sink.count, 1)

        pubs_before = self._publishers(HARDWARE_TOPIC)
        self.assertEqual(1, len(pubs_before), pubs_before)

        # Separate process/node so master sees a second publisher identity.
        rogue = subprocess.Popen(
            [
                "rostopic", "pub", "-r", "20", HARDWARE_TOPIC,
                "geometry_msgs/Twist", "{linear: {x: 0.01}}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            deadline = time.time() + 3.0
            pubs = pubs_before
            while time.time() < deadline:
                pubs = self._publishers(HARDWARE_TOPIC)
                if len(pubs) >= 2:
                    break
                time.sleep(0.1)
            self.assertGreaterEqual(len(pubs), 2, pubs)
        finally:
            rogue.terminate()
            try:
                rogue.wait(timeout=2)
            except Exception:
                rogue.kill()

    @staticmethod
    def _publishers(topic):
        import rosgraph
        master = rosgraph.Master(rospy.get_name())
        state = master.getSystemState()
        publishers = state[0]
        for name, nodes in publishers:
            if name == topic:
                return list(nodes)
        return []


if __name__ == "__main__":
    rospy.init_node("test_ranger_topic_isolation")
    rostest.rosrun(
        "remani_real", "test_ranger_topic_isolation", RangerTopicIsolationTest)
# ################################
# Python: Ranger topic isolation rostest end
# ################################
