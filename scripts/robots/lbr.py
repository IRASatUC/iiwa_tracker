#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition


class iiwaRobot():
    def __init__(self):
        rospy.init_node('iiwa_node',anonymous=True, log_level=rospy.DEBUG)
        self.jpos_publisher = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)
        self.rate = rospy.Rate(30.0)
        # tf listener
        self.tf_listener = tf.TransformListener()

    def pub_joint_pos(self, joint_position=JointPosition()):
        self.jpos_publisher.publish(joint_position)
        rospy.loginfo("iiwa is moving to {}".format(joint_position))
