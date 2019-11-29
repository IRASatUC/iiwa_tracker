#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import sys
import os
import time

import cv2
import pyrealsense2 as rs
import numpy as np
from numpy import pi
from glob import glob

import torch

from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker

import rospy
import tf
from robots.lbr import iiwaRobot
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition


# Define constant perching position
JOINT_PERCH = JointPosition()
JOINT_PERCH.position.a2 = pi/6
JOINT_PERCH.position.a4 = -pi/3
JOINT_PERCH.position.a6 = pi/3


def main():
    # instantiate iiwa
    iiwa = iiwaRobot()
    time.sleep(4)
    # zero joints
    for _ in range(20):
        iiwa.pub_joint_pos()
    time.sleep(4)
    # iiwa get ready
    for _ in range(20):
        iiwa.pub_joint_pos(JOINT_PERCH)
    time.sleep(4)
    rospy.loginfo("iiwa is ready")
    # Configure realsense D435 depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    # load siammask config
    cfg.merge_from_file(sys.path[0]+"/siammask_r50_l3/config.yaml")
    cfg.CUDA = torch.cuda.is_available()
    device = torch.device('cuda' if cfg.CUDA else 'cpu')
    # create model
    model = ModelBuilder()
    # load model
    model.load_state_dict(torch.load(sys.path[0]+"/siammask_r50_l3/model.pth",
        map_location=lambda storage, loc: storage.cpu()))
    model.eval().to(device)
    # build tracker
    tracker = build_tracker(model)
    # label object
    first_frame = True
    video_name = 'realsense_D435'
    cv2.namedWindow(video_name, cv2.WND_PROP_FULLSCREEN)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
