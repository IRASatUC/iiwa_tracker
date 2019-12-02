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

from robots import utils

# iiwa's initial perching pose
JOINT_PERCH = JointPosition()
JOINT_PERCH.position.a2 = pi/6
JOINT_PERCH.position.a4 = -pi/3
JOINT_PERCH.position.a6 = pi/3

iiwa = iiwaRobot()
print(iiwa.joint_position)
time.sleep(4) # allow iiwa taking some time to wake up
print(iiwa.joint_position)
# zero joints
# for _ in range(20):
iiwa.move_joint()
time.sleep(4)
# iiwa get ready
# for _ in range(20):
iiwa.move_joint(JOINT_PERCH)
time.sleep(4)
print("READY")

print("Current cartesian pose: \n{}".format(iiwa.cartesian_pose))
# iiwa.goal_carte_pose.pose = iiwa.cartesian_pose
iiwa.goal_carte_pose.header.frame_id = 'iiwa_link_0'
iiwa.goal_carte_pose.pose.position.x = iiwa.cartesian_pose.position.x-0.2
iiwa.goal_carte_pose.pose.position.y = iiwa.cartesian_pose.position.y-0.1
iiwa.goal_carte_pose.pose.position.z = iiwa.cartesian_pose.position.z+0.2
iiwa.goal_carte_pose.pose.orientation.x = iiwa.cartesian_pose.orientation.x
iiwa.goal_carte_pose.pose.orientation.y = iiwa.cartesian_pose.orientation.y
iiwa.goal_carte_pose.pose.orientation.z = iiwa.cartesian_pose.orientation.z
iiwa.goal_carte_pose.pose.orientation.w = iiwa.cartesian_pose.orientation.w
iiwa.move_cartesian(iiwa.goal_carte_pose, commit=True)
# while not iiwa.goal_approximation(type='cp'):
#     iiwa.move_cartesian(iiwa.goal_carte_pose)
#     print("iiwa at goal? \n{}".format(iiwa.goal_approximation(type='cp')))
# print("iiwa at goal? \n{}".format(iiwa.goal_approximation(type='cp')))

# iiwa.goal_carte_pose.pose.position.x += 0.2
# iiwa.goal_carte_pose.pose.position.y += 0.2
# iiwa.goal_carte_pose.pose.position.z -= 0.2
# iiwa.move_cartesian(iiwa.goal_carte_pose, commit=True)
# while not iiwa.goal_approximation(type='cp'):
#     print("Stoned: {}".format(np.allclose(utils.jq_to_array(iiwa.joint_velocity), np.zeros(7),atol=1e-04)))
#
# while not rospy.is_shutdown():
#     iiwa.goal_carte_pose = PoseStamped()
#     iiwa.goal_carte_pose.pose = iiwa.cartesian_pose
#     # print("Robot joint state: \n{} \nRobot cartesian state: \n{}".format(iiwa.joint_position, iiwa.cartesian_pose))
#     rospy.logwarn("@Goal? {}".format(iiwa.goal_approximation(type='cp')))
