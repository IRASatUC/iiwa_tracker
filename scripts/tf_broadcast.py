#!/usr/bin/env python
import roslib
roslib.load_manifest('iiwa_gazebo')

import rospy
import tf
from math import pi

# child of iiwa_link_0
YAW, PITCH, ROLL = -pi/2, 0.00, -pi/2
ROT = tf.transformations.quaternion_from_euler(YAW, PITCH, ROLL, 'rzyx')
TRANS = (0.208, 0., 0.175)

# child of iiwa_link_ee
# YAW, PITCH, ROLL = -pi/2, 0.010, -0.012
# ROT = tf.transformations.quaternion_from_euler(YAW, PITCH, ROLL, 'rzyx')
# TRANS = (-0.03, 0.0175, 0.03880)
if __name__ == '__main__':
    rospy.init_node('tf_broadcast_node')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        br.sendTransform(TRANS,
                         ROT,
                         rospy.Time.now(),
                         "rs_d435",
                         "iiwa_link_0")
        br.sendTransform((0., 0., 0.167),
                         (0., 0., 0., 1.),
                         rospy.Time.now(),
                         "iiwa_tcp",
                         "iiwa_link_ee")
        rate.sleep()
