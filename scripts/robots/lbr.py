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
from iiwa_msgs.msg import JointQuantity, JointPosition, JointVelocity, CartesianPose

from robots import utils

RATE = 30

class iiwaRobot(object):
    def __init__(self):
        rospy.init_node('iiwa_node',anonymous=True, log_level=rospy.INFO)
        self.rate = rospy.Rate(RATE)
        self.joint_position = JointQuantity()
        self.joint_velocity = JointQuantity()
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
        # subscriber
        rospy.Subscriber('iiwa/state/JointVelocity', JointVelocity, self._jvel_cb)

        super(iiwaRobot, self).__init__()

    def move_joint(self, joint_position=JointPosition(), commit=False):
        assert joint_position._type == 'iiwa_msgs/JointPosition'
        self.joint_pos_publisher.publish(joint_position)
        rospy.logwarn("iiwa is moving to joint position: \n{}".format(joint_position))
        if commit:
            num_stones = 0
            while not self.goal_approximation(type='jp'):
                rospy.logdebug("Goal not reached yet...")
                if np.allclose(utils.jq_to_array(self.joint_velocity), np.zeros(7),atol=1e-04):
                    num_stones += 1
                else:
                    num_stones = 0
                # if robot not move in a duration of 1 sec, stop moving
                if num_stones >= RATE:
                    rospy.logerr("Goal is not reachable")
                    break
                self.joint_pos_publisher.publish(joint_position)
                self.rate.sleep()

    def move_cartesian(self, cartesian_pose, commit=False):
        assert cartesian_pose._type == 'geometry_msgs/PoseStamped'
        self.carte_pose_publisher.publish(cartesian_pose)
        rospy.loginfo("iiwa is moving to cartesian pose: \n{}".format(cartesian_pose))
        if commit:
            num_stones = 0
            while not self.goal_approximation(type='cp'):
                rospy.logdebug("Goal not reached yet...")
                if np.allclose(utils.jq_to_array(self.joint_velocity), np.zeros(7),atol=1e-04):
                    num_stones += 1
                else:
                    num_stones = 0
                # if robot not move in a duration of 1 sec, stop moving
                if num_stones >= RATE:
                    rospy.logerr("Goal is not reachable")
                    break
                self.carte_pose_publisher.publish(cartesian_pose)
                self.rate.sleep()

    def goal_approximation(self, type, threshold=1e-04):
        if type=='jp':
            goal_jpos = np.array([
                self.goal_joint_pos.position.a1,
                self.goal_joint_pos.position.a2,
                self.goal_joint_pos.position.a3,
                self.goal_joint_pos.position.a4,
                self.goal_joint_pos.position.a5,
                self.goal_joint_pos.position.a6,
                self.goal_joint_pos.position.a7
            ])
            curr_jpos = np.array([
                self.joint_position.a1,
                self.joint_position.a2,
                self.joint_position.a3,
                self.joint_position.a4,
                self.joint_position.a5,
                self.joint_position.a6,
                self.joint_position.a7
            ])
            dist_jpos = np.linalg.norm(goal_jpos-curr_jpos)

            return dist_jpos <= threshold
        elif type=='cp':
            # convert pose to array
            goal_cpos = np.array([
                self.goal_carte_pose.pose.position.x,
                self.goal_carte_pose.pose.position.y,
                self.goal_carte_pose.pose.position.z
            ])
            goal_cquat = np.array([
                self.goal_carte_pose.pose.orientation.x,
                self.goal_carte_pose.pose.orientation.y,
                self.goal_carte_pose.pose.orientation.z,
                self.goal_carte_pose.pose.orientation.w,
            ])
            curr_cpos = np.array([
                self.cartesian_pose.position.x,
                self.cartesian_pose.position.y,
                self.cartesian_pose.position.z
            ])
            curr_cquat = np.array([
                self.cartesian_pose.orientation.x,
                self.cartesian_pose.orientation.y,
                self.cartesian_pose.orientation.z,
                self.cartesian_pose.orientation.w,
            ])
            # euclidean distance
            dist_cpos = np.linalg.norm(goal_cpos-curr_cpos)
            # diff quaternion
            diff_cquat = 1-abs(np.dot(goal_cquat,curr_cquat))

            return dist_cpos <= threshold and diff_cquat <= threshold

    def _jpos_cb(self, data):
        self.joint_position = data.position
        rospy.logdebug("robot joint position: {}".format(self.joint_position))

    def _jvel_cb(self, data):
        self.joint_velocity = data.velocity
        rospy.logdebug("robot joint velocity: {}".format(self.joint_velocity))

    def _cpose_cb(self, data):
        self.cartesian_pose = data.poseStamped.pose
        rospy.logdebug("robot cartesian pose: {}".format(self.cartesian_pose))
