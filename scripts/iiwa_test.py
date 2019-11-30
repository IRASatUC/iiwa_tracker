#!/usr/bin/env python
""" iiwa robot class
"""
from __future__ import absolute_import, division, print_function

import sys
import time
import numpy as np

import rospy
import tf
from robots.lbr import iiwaRobot
from geometry_msgs.msg import Pose, PoseStamped
from iiwa_msgs.msg import JointQuantity, JointPosition, CartesianPose


iiwa = iiwaRobot()

while not rospy.is_shutdown():
    iiwa.goal_carte_pose = PoseStamped()
    iiwa.goal_carte_pose.pose = iiwa.cartesian_pose
    # print("Robot joint state: \n{} \nRobot cartesian state: \n{}".format(iiwa.joint_position, iiwa.cartesian_pose))
    rospy.logwarn("@Goal? {}".format(iiwa.goal_approximation(mode='cp')))
