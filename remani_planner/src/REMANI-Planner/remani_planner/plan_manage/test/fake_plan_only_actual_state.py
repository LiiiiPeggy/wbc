#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


JOINTS_DEG = [0.0, -40.0, 130.0, 0.0, 30.0, 0.0]


def main():
    rospy.init_node("fake_plan_only_actual_state")
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
    joint_pub = rospy.Publisher("/joint_state", JointState, queue_size=1)
    gripper_pub = rospy.Publisher("/gripper_state", Bool, queue_size=1)
    trigger_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped,
                                  queue_size=1)

    rate = rospy.Rate(50)
    ready_ticks = 0
    triggered = False
    while not rospy.is_shutdown():
        now = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = now
        joints.name = ["joint%d" % (i + 1) for i in range(6)]
        joints.position = [math.radians(value) for value in JOINTS_DEG]
        joints.velocity = [0.0] * 6
        joints.effort = [0.0] * 6
        joint_pub.publish(joints)
        gripper_pub.publish(Bool(data=False))

        connected = (odom_pub.get_num_connections() > 0 and
                     joint_pub.get_num_connections() > 0 and
                     trigger_pub.get_num_connections() > 0 and
                     rospy.get_param("/test_remani_plan_only/ready", False))
        ready_ticks = ready_ticks + 1 if connected else 0
        if not triggered and ready_ticks >= 50:
            goal = PoseStamped()
            goal.header.stamp = now
            goal.header.frame_id = "world"
            goal.pose.orientation.w = 1.0
            trigger_pub.publish(goal)
            triggered = True

        rate.sleep()


if __name__ == "__main__":
    main()
