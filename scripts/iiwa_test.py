#!/usr/bin/env python
""" iiwa tests
"""
from __future__ import absolute_import, division, print_function

import sys
import time
import numpy as np
from numpy import pi

import rospy
import tf
from robots.lbr import iiwaRobot
from geometry_msgs.msg import Pose, PoseStamped
from iiwa_msgs.msg import JointQuantity, JointPosition, CartesianPose


# iiwa's initial perching pose
JOINT_PERCH = JointPosition()
JOINT_PERCH.position.a2 = pi/6
JOINT_PERCH.position.a4 = -pi/3
JOINT_PERCH.position.a6 = pi/3

iiwa = iiwaRobot()
time.sleep(4) # allow iiwa taking some time to wake up
# zero joints
for _ in range(20):
    iiwa.move_joint()
time.sleep(4)
# iiwa get ready
for _ in range(20):
    iiwa.move_joint(JOINT_PERCH)
time.sleep(8)
print("READY")

iiwa.goal_carte_pose.pose = iiwa.cartesian_pose
iiwa.goal_carte_pose.header.frame_id = 'iiwa_link_0'
iiwa.goal_carte_pose.pose.position.x += 0.04
iiwa.goal_carte_pose.pose.position.x -= 0.02
iiwa.goal_carte_pose.pose.position.x += 0.04

# for _ in range(20):
iiwa.move_cartesian(iiwa.goal_carte_pose)
# while not rospy.is_shutdown():
#     iiwa.goal_carte_pose = PoseStamped()
#     iiwa.goal_carte_pose.pose = iiwa.cartesian_pose
#     # print("Robot joint state: \n{} \nRobot cartesian state: \n{}".format(iiwa.joint_position, iiwa.cartesian_pose))
#     rospy.logwarn("@Goal? {}".format(iiwa.goal_approximation(mode='cp')))
