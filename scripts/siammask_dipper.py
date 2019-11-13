#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import sys
import os

import cv2
import pyrealsense2 as rs
import numpy as np
from numpy import pi
from glob import glob

import torch
import tensorrt as trt
from torch2trt import tensorrt_converter

from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker

import rospy
from iiwa_msgs.msg import JointPosition

# Define constant perching position
JOINT_PERCH = JointPosition()
JOINT_PERCH.position.a2 = pi/9
JOINT_PERCH.position.a4 = -7*pi/18
JOINT_PERCH.position.a6 = 7*pi/18

# Configure realsense D435 depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

def iiwa_transform():
    # vicon frame is rotated 90 degrees along z axis in iiwa frame
    theta = np.radians(90)
    rot = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])
    # vicon frame origin in iiwa frame (1.825, 0.385, 0.065) m
    trans = np.array([1.85,0.35,-0.07])

    return rot, trans

class iiwaRobot():
    def __init__(self):
        rospy.init_node('iiwa_node',anonymous=True, log_level=rospy.DEBUG)
        self.jpos_publisher = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)

    def pub_joint_pos(self, joint_position):
        if not joint_position:
            joint_position = JointPosition()
        jpos.publish(joint_position)

def main():
    # initialize
    iiwa = iiwaRobot()
    iiwa.pub_joint_pos()
    time.sleep(4)
    # get ready
    iiwa.pub_joint_pos(JOINT_PERCH)
    time.sleep(1)
    rospy.loginfo("iiwa is ready")

    # load siammask config
    cfg.merge_from_file(sys.path[0]+"siammask_r50_l3/config.yaml")
    cfg.CUDA = torch.cuda.is_available()
    device = torch.device('cuda' if cfg.CUDA else 'cpu')
    # create model
    model = ModelBuilder()
    # load model
    model.load_state_dict(torch.load(sys.path[0]+"siammask_r50_l3/model.pth",
        map_location=lambda storage, loc: storage.cpu()))
    model.eval().to(device)
    # build tracker
    tracker = build_tracker(model)
    # label object
    first_frame = True
    video_name = 'realsense_D435'
    cv2.namedWindow(video_name, cv2.WND_PROP_FULLSCREEN)
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        # convert image to numpy arrays
        if color_frame:
            frame = np.asanyarray(color_frame.get_data())
        if first_frame:
            try:
                init_rect = cv2.selectROI(video_name, frame, False, False)
            except:
                exit()
            tracker.init(frame, init_rect)
            first_frame = False
        # start tracking
        else:
            outputs = tracker.track(frame)
            if 'polygon' in outputs:
                polygon = np.array(outputs['polygon']).astype(np.int32)
                cv2.polylines(frame, [polygon.reshape((-1, 1, 2))],
                              True, (0, 255, 0), 3)
                mask = ((outputs['mask'] > cfg.TRACK.MASK_THERSHOLD) * 255)
                mask = mask.astype(np.uint8)
                mask = np.stack([mask, mask*255, mask]).transpose(1, 2, 0)
                frame = cv2.addWeighted(frame, 0.77, mask, 0.23, -1)
            else:
                bbox = list(map(int, outputs['bbox']))
                cv2.rectangle(frame, (bbox[0], bbox[1]),
                              (bbox[0]+bbox[2], bbox[1]+bbox[3]),
                              (0, 255, 0), 3)
                x_of_obj = bbox[0]+0.5*bbox[2]
                y_of_obj = bbox[1]+0.5*bbox[3]


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
