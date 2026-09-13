#!/usr/bin/env python3
# ################################
# Python: fake Ranger driver for staged gate begin
# ################################
from __future__ import print_function

import math
import threading

import rospy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty


class FakeRangerDriver(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd_time = None
        self.timeout = rospy.get_param("~cmd_vel_timeout", 0.20)
        self.timed_out = False
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.ready_pub = rospy.Publisher(
            "/remani/ranger_watchdog_ready", Bool, queue_size=1, latch=True)
        self.timed_out_pub = rospy.Publisher(
            "/remani/ranger_watchdog_timed_out", Bool, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(
            "/remani/hardware/ranger/cmd_vel", Twist, self.on_cmd, queue_size=10)
        rospy.Subscriber("/remani/test/reset_ranger", Empty, self.on_reset, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.on_timer)
        self.ready_pub.publish(Bool(data=True))
        self.timed_out_pub.publish(Bool(data=False))
        rospy.loginfo("fake_ranger_driver online on /remani/hardware/ranger/cmd_vel")

    def on_reset(self, _msg):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.vx = 0.0
            self.wz = 0.0
            # Do not arm watchdog until the first hardware command arrives.
            self.last_cmd_time = None
            if self.timed_out:
                self.timed_out = False
                self.timed_out_pub.publish(Bool(data=False))

    def on_cmd(self, msg):
        now = rospy.Time.now()
        with self.lock:
            self.vx = msg.linear.x
            self.wz = msg.angular.z
            self.last_cmd_time = now
            if self.timed_out:
                self.timed_out = False
                self.timed_out_pub.publish(Bool(data=False))

    def on_timer(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if self.last_cmd_time is not None:
                age = (now - self.last_cmd_time).to_sec()
                if age > self.timeout:
                    self.vx = 0.0
                    self.wz = 0.0
                    if not self.timed_out:
                        self.timed_out = True
                        self.timed_out_pub.publish(Bool(data=True))
            dt = 0.05
            self.yaw += self.wz * dt
            self.x += self.vx * math.cos(self.yaw) * dt
            self.y += self.vx * math.sin(self.yaw) * dt
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "world"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = Quaternion(
                0.0, 0.0, math.sin(self.yaw * 0.5), math.cos(self.yaw * 0.5))
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.angular.z = self.wz
            self.odom_pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("fake_ranger_driver")
    FakeRangerDriver()
    rospy.spin()
# ################################
# Python: fake Ranger driver for staged gate end
# ################################
