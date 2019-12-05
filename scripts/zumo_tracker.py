#!/usr/bin/env python

from __future__ import absolute_import, print_function

import rospy
from geometry_msgs.msg import TransformStamped

class ViconBridge(object):
    def __init__(self, node_name, log_level=rospy.DEBUG):
        """
        A vicon_bridge class
        """
        # zumopi position
        self.zumo_pos = TransformStamped().transform
        rospy.init_node(node_name, anonymous=True, log_level=log_level)

        rospy.Subscriber('/vicon/zumopi/zumopi', TransformStamped, self._zumo_pos_callback)
        #rospy.logdebug("vicon tracker initiated.")

    def _zumo_pos_callback(self, data):
        self.zumo_pos = data.transform
        #rospy.logdebug("zumo at:{}".format(self.zumo_pos))
