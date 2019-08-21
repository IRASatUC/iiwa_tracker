#!/usr/bin/env python
#license remove for brevity

import rospy
import numpy as np
from iiwa_msgs.msg import JointPosition
from tracker import ViconBridge

# # def JointAngleA1(a1):
#     if a1 == 0.0:
#         a1 = 0.8
#     else:
#         a1 = 0.0
#     return a1

def iiwa_transform():
    # vicon frame is rotated 90 degrees along z axis in iiwa frame
    theta = np.radians(90)
    rot = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])
    # vicon frame origin in iiwa frame (1.825, 0.385, 0.065) m
    trans = np.array([1.85,0.35,-0.07])

    return rot, trans

def iiwa_joint_position(zumoPos):
    # transform zumo to iiwa frame
    rot, trans = iiwa_transform()
    zpos = np.array((zumoPos.translation.x,
                        zumoPos.translation.y,
                        zumoPos.translation.z))
    pos = np.dot(rot,zpos)+trans
    print("zumo position:{}".format(pos))
    x = pos[0]
    y = pos[1]
    z = pos[2]

    """
    iiwa robot gesture '0' is camera, '+'' rotate joint
    o-+---
         |
         |
         |
    """
    # limit axis 1 in [-159,159]
    ang1 = np.arctan2(y, x)
    if ang1 <= -160*np.pi/180:
        ang1 = -159*np.pi/180
    elif ang1 >= 160*np.pi/180:
        ang1 = 159

    # fix axis 4 as -pi/2
    ang4 = -0.5*np.pi

    # limit axis 6 in [-109, 109]
    dist = np.sqrt(x*x+y*y)
    ang6 =  np.arctan2(0.78-z, dist-0.4)
    if ang6 <= -110*np.pi/180:
        ang6 = -109*np.pi/180
    elif ang6 >= 110*np.pi/180:
        ang6 = 109*np.pi/180

    jointPos = JointPosition()
    jointPos.position.a1 = ang1
    jointPos.position.a2 = 0.0
    jointPos.position.a3 = 0.0
    jointPos.position.a4 = ang4
    jointPos.position.a5 = 0.0
    jointPos.position.a6 = ang6
    jointPos.position.a7 = 0.0

    print("iiwa joint position(a1,a6):({},{})".format(ang1, ang6))
    print
    return jointPos

def controller():
    # create a tracker node with vicon
    # get the position of the zumopi
    tracker = ViconBridge("vicon tracker")
    print(tracker.zumo_pos)
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)
    rate = rospy.Rate(2)

    last_a1, last_a6 = 0.0, 0.0
    # a1 = 0.0
    while not rospy.is_shutdown():
        jointPos = iiwa_joint_position(tracker.zumo_pos)
        a1, a6 = jointPos.position.a1, jointPos.position.a6
        if a1 != last_a1 or a6 != last_a6:
            pub.publish(jointPos)
            last_a1, last_a6 = a1, a6
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
