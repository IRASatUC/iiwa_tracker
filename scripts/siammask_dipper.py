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
# import tensorrt as trt
# from torch2trt import tensorrt_converter

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
JOINT_PERCH.position.a2 = pi/9
JOINT_PERCH.position.a4 = -7*pi/18
JOINT_PERCH.position.a6 = 7*pi/18

# Configure realsense D435 depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)


def main():
    # initialize
    iiwa = iiwaRobot()
    for _ in range(5):
        iiwa.pub_joint_pos(JointPosition())
        time.sleep(.1)
    # get ready
    iiwa.pub_joint_pos(JOINT_PERCH)
    time.sleep(5)
    rospy.loginfo("iiwa is ready")

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
                bbox = list(map(int, outputs['bbox']))
                cv2.rectangle(frame, (bbox[0], bbox[1]),
                              (bbox[0]+bbox[2], bbox[1]+bbox[3]),
                              (0, 255, 0), 3)
                x_of_obj = bbox[0]+0.5*bbox[2]
                y_of_obj = bbox[1]+0.5*bbox[3]

                depth_pixel = [x_of_obj, y_of_obj]
                depth_3d = depth_frame.get_distance(int(x_of_obj), int(y_of_obj))
                point3d = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_3d)
                print("Object 3D position: {}".format(point3d))
                t = iiwa.rs_ls.getLatestCommonTime('/iiwa_link_0', '/rs_d435')
                poi = PoseStamped()
                poi.header.frame_id = 'rs_d435'
                poi.pose.orientation.w = 1.
                poi.pose.position.x = point3d[0]
                poi.pose.position.y = point3d[1]
                poi.pose.position.z = point3d[2]
                point_in_iiwa = iiwa.rs_ls.transformPose('/iiwa_link_0', poi)
                rospy.loginfo("Point in iiwa: {}".format(point_in_iiwa))
            # else:
            #     bbox = list(map(int, outputs['bbox']))
            #     cv2.rectangle(frame, (bbox[0], bbox[1]),
            #                   (bbox[0]+bbox[2], bbox[1]+bbox[3]),
            #                   (0, 255, 0), 3)
            #     x_of_obj = bbox[0]+0.5*bbox[2]
            #     y_of_obj = bbox[1]+0.5*bbox[3]
            #
            #     depth_pixel = [x_of_obj, y_of_obj]
            #     depth_3d = depth_frame.get_distance(int(x_of_obj), int(y_of_obj))
            #     point3d = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_3d)
            #     print("x:{}, y: {}, z:{}".format(x_of_obj,y_of_obj,depth_3d))

            cv2.imshow(video_name, frame)
            key = cv2.waitKey(40)
            if key in (27, ord("q")):
                break

    pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
