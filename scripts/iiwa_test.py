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
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from iiwa_msgs.msg import JointQuantity, JointPosition, CartesianPose

from robots import utils

# iiwa's initial perching pose
JOINT_PERCH = JointPosition()
JOINT_PERCH.position.a1 = -24.51/180*pi
JOINT_PERCH.position.a2 = 31.57/180*pi
JOINT_PERCH.position.a3 = pi/6
JOINT_PERCH.position.a4 = -106.86/180*pi
JOINT_PERCH.position.a5 = -0.51/180*pi
JOINT_PERCH.position.a6 = -46.06/180*pi
JOINT_PERCH.position.a7 = -42.96/180*pi
# square corners
C1, C2, C3, C4 = Point(), Point(), Point(), Point()
x = 0.85
y1, z1 = -0.35, 0.3
C1.x, C1.y, C1.z = x, y1, z1
y2, z2 = 0.35, 0.3
C2.x, C2.y, C2.z = x, y2, z2
y3, z3 = 0.35, 0.7
C3.x, C3.y, C3.z = x, y3, z3
y4, z4 = -0.35, 0.7
C4.x, C4.y, C4.z = x, y4, z4


iiwa = iiwaRobot()
time.sleep(4) # allow iiwa taking some time to wake up
iiwa.move_joint(commit=True)
time.sleep(1)
# iiwa get ready
iiwa.move_joint(JOINT_PERCH, commit=True)
print("iiwa is ready")
time.sleep(4)
# read TCP orientation
QUAT = Quaternion()
QUAT.x = iiwa.cartesian_pose.orientation.x
QUAT.y = iiwa.cartesian_pose.orientation.y
QUAT.z = iiwa.cartesian_pose.orientation.z
QUAT.w = iiwa.cartesian_pose.orientation.w
print("Current cartesian pose: \n{}".format(iiwa.cartesian_pose))
# iiwa.goal_carte_pose.pose = iiwa.cartesian_pose
iiwa.goal_carte_pose.header.frame_id = 'iiwa_link_0'
iiwa.goal_carte_pose.pose.position = C1
iiwa.goal_carte_pose.pose.orientation = QUAT
iiwa.move_cartesian(iiwa.goal_carte_pose, commit=True)
time.sleep(2)
iiwa.goal_carte_pose.pose.position = C2
iiwa.goal_carte_pose.pose.orientation = QUAT
iiwa.move_cartesian(iiwa.goal_carte_pose, commit=True)
time.sleep(2)
iiwa.goal_carte_pose.pose.position = C3
iiwa.goal_carte_pose.pose.orientation = QUAT
iiwa.move_cartesian(iiwa.goal_carte_pose, commit=True)
time.sleep(2)
iiwa.goal_carte_pose.pose.position = C4
iiwa.goal_carte_pose.pose.orientation = QUAT
iiwa.move_cartesian(iiwa.goal_carte_pose, commit=True)
time.sleep(2)
iiwa.goal_carte_pose.pose.position = C1
iiwa.goal_carte_pose.pose.orientation = QUAT
iiwa.move_cartesian(iiwa.goal_carte_pose, commit=True)

time.sleep(2)
iiwa.move_joint(JOINT_PERCH, commit=True)
print("Finished!")
