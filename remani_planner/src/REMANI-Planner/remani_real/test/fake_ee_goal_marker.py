#!/usr/bin/env python3
# ################################
# Python: fake EE goal marker begin
# ################################
"""Publish a fixed reachable EE goal whenever /ee_goal_plan is signaled."""

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty


class FakeEeGoalMarker(object):
    def __init__(self):
        self._goal = PoseStamped()
        self._goal.header.frame_id = "map"
        self._goal.pose.position.x = 0.5
        self._goal.pose.position.y = 0.0
        self._goal.pose.position.z = 0.8
        self._goal.pose.orientation.w = 1.0

        self._pub = rospy.Publisher("/ee_goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/ee_goal_plan", Empty, self._on_plan, queue_size=10)

    def _on_plan(self, _msg):
        self._goal.header.stamp = rospy.Time.now()
        self._pub.publish(self._goal)
        rospy.loginfo("Published fixed /ee_goal at (0.5, 0.0, 0.8) in map")


def main():
    rospy.init_node("fake_ee_goal_marker")
    FakeEeGoalMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
# ################################
# Python: fake EE goal marker end
# ################################
