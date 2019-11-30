#!/usr/bin/env python
""" iiwa robot class
"""
from __future__ import absolute_import, division, print_function

import sys
import time
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from iiwa_msgs.msg import JointQuantity, JointPosition, CartesianPose


class iiwaRobot(object):
    def __init__(self):
        rospy.init_node('iiwa_node',anonymous=True, log_level=rospy.DEBUG)
        self.rate = rospy.Rate(30.0)
        self.joint_position = JointQuantity()
        self.cartesian_pose = Pose()
        self.goal_joint_pos = JointPosition()
        self.goal_carte_pose = PoseStamped()
        # tf listener
        self.tf_listener = tf.TransformListener()
        # publisher
        self.joint_pos_publisher = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)
        self.carte_pose_publisher = rospy.Publisher('iiwa/command/CartesianPose', PoseStamped, queue_size=1)
        # subscriber
        rospy.Subscriber('iiwa/state/JointPosition', JointPosition, self._jpos_cb)
        rospy.Subscriber('iiwa/state/CartesianPose', CartesianPose, self._cpose_cb)

        super(iiwaRobot, self).__init__()

    def move_joint(self, joint_position=JointPosition(), commit=False):
        assert joint_position._type == 'iiwa_msgs/JointPosition'
        self.joint_pos_publisher.publish(joint_position)
        rospy.loginfo("iiwa is moving to {}".format(joint_position))

    def move_cartesian(self, cartesian_pose, commit=False):
        assert cartesian_pose._type == 'geometry_msgs/PoseStamped'
        if commit:
            while not self.goal_approximation(mode='cp'):
                self.carte_pose_publisher.publish(cartesian_pose)
                self.rate.sleep()
        else:
            self.carte_pose_publisher.publish(cartesian_pose)
        rospy.logdebug("robot toward: {}".format(cartesian_pose))

    def goal_approximation(self, mode, threshold=0.01):
        if mode=='cp':
            # convert pose to array
            goal_pos = np.array([
                self.goal_carte_pose.pose.position.x,
                self.goal_carte_pose.pose.position.y,
                self.goal_carte_pose.pose.position.z
            ])
            goal_quat = np.array([
                self.goal_carte_pose.pose.orientation.x,
                self.goal_carte_pose.pose.orientation.y,
                self.goal_carte_pose.pose.orientation.z,
                self.goal_carte_pose.pose.orientation.w,
            ])
            curr_pos = np.array([
                self.cartesian_pose.position.x,
                self.cartesian_pose.position.y,
                self.cartesian_pose.position.z
            ])
            curr_quat = np.array([
                self.cartesian_pose.orientation.x,
                self.cartesian_pose.orientation.y,
                self.cartesian_pose.orientation.z,
                self.cartesian_pose.orientation.w,
            ])
            # euclidean distance
            dist_pos = np.linalg.norm(goal_pos-curr_pos)
            # diff quaternion
            diff_quat = 1-abs(np.dot(goal_quat,curr_quat))

            return dist_pos <= threshold and diff_quat <= threshold

    def _jpos_cb(self, data):
        self.joint_position = data.position
        rospy.logdebug("robot joint position: {}".format(self.joint_position))

    def _cpose_cb(self, data):
        self.cartesian_pose = data.poseStamped.pose
        rospy.logdebug("robot cartesian pose: {}".format(self.cartesian_pose))
