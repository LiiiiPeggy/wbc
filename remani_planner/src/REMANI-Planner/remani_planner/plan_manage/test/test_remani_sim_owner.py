#!/usr/bin/env python3

import threading
import unittest

import rosnode
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PolynomialTraj


class RemaniSimOwnerTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._received_add_count = 0
        self._received_start_count = 0
        self._received_final_count = 0
        self._trajectory_sub = rospy.Subscriber(
            "/planning/trajectory", PolynomialTraj,
            self._trajectory_callback, queue_size=100)
        self._goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)

    def _trajectory_callback(self, message):
        with self._lock:
            if message.action == PolynomialTraj.ACTION_ADD:
                self._received_add_count += 1
            elif message.action == PolynomialTraj.ACTION_WARN_START:
                self._received_start_count += 1
            elif message.action == PolynomialTraj.ACTION_WARN_FINAL:
                self._received_final_count += 1

    @staticmethod
    def _controller_subscribes_to_trajectory():
        _code, _message, state = rospy.get_master().getSystemState()
        for topic, subscribers in state[1]:
            if topic == "/planning/trajectory":
                return "/mm_controller_node" in subscribers
        return False

    def test_default_sim_uses_internal_controller_add_stream(self):
        self.assertEqual("sim", rospy.get_param("/remani_planner_node/mode", "sim"))
        self.assertEqual("internal", rospy.get_param(
            "/remani_planner_node/execution_owner", "internal"))

        controller_deadline = rospy.Time.now() + rospy.Duration(30.0)
        while rospy.Time.now() < controller_deadline:
            if ("/mm_controller_node" in rosnode.get_node_names() and
                    self._controller_subscribes_to_trajectory()):
                break
            rospy.sleep(0.1)

        self.assertIn("/mm_controller_node", rosnode.get_node_names())
        self.assertTrue(self._controller_subscribes_to_trajectory())

        goal_connection_deadline = rospy.Time.now() + rospy.Duration(30.0)
        while (self._goal_pub.get_num_connections() == 0 and
               rospy.Time.now() < goal_connection_deadline):
            rospy.sleep(0.1)
        self.assertGreater(
            self._goal_pub.get_num_connections(), 0,
            "planner did not subscribe to the existing public goal topic")

        # The Ranger+CR10 wrapper defaults target_type=1, so this is a normal
        # manual 2D goal.  The map reserves 3 m clear zones around the initial
        # state (-5, 0) and this goal (-2.5, 0); the path stays west of the
        # configured bridge at x=0.
        # Wait briefly after the subscriber appears so the FSM can leave INIT
        # on the simulator's odometry and accept the trigger in WAIT_TARGET.
        rospy.sleep(1.0)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = -2.5
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)

        add_deadline = rospy.Time.now() + rospy.Duration(120.0)
        while rospy.Time.now() < add_deadline:
            with self._lock:
                if self._received_add_count > 0:
                    break
            rospy.sleep(0.1)

        with self._lock:
            received_add_count = self._received_add_count
            received_start_count = self._received_start_count
            received_final_count = self._received_final_count

        self.assertGreater(received_add_count, 0)
        self.assertEqual(0, received_start_count)
        self.assertEqual(0, received_final_count)


if __name__ == "__main__":
    rospy.init_node("test_remani_sim_owner")
    rostest.rosrun("remani_planner", "test_remani_sim_owner",
                   RemaniSimOwnerTest)
