#!/usr/bin/env python
""" Transform POI from realsense frame into iiwa frame
    Pre-requisite:
        roslaunch iiwa_gazebo iiwa_gazebo_with_sunrise.launch (tf defined in that package)
"""
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
from glob import glob

import torch

from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker

import rospy
import tf
from robots.lbr import iiwaRobot
from geometry_msgs.msg import PoseStamped


def main():
    # instantiate iiwa
    # iiwa = iiwaRobot()
    # time.sleep(4) # allow iiwa taking some time to wake up
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
    # begin tracking
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
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
                poi_pixel = [int(bbox[0]+0.5*bbox[2]), int(bbox[1]+0.5*bbox[3])]
                poi_depth = depth_frame.get_distance(poi_pixel[0], poi_pixel[1])
                poi_rs = rs.rs2_deproject_pixel_to_point(depth_intrinsics, poi_pixel, poi_depth)
                rospy.logdebug("Object 3D position w.r.t. camera frame: {}".format(poi_rs))
                # transfrom = iiwa.tf_listener.getLatestCommonTime('/iiwa_link_0', '/rs_d435')
                # pos_rs = PoseStamped()
                # pos_rs.header.frame_id = 'rs_d435'
                # pos_rs.pose.orientation.w = 1.
                # pos_rs.pose.position.x = poi_rs[0]
                # pos_rs.pose.position.y = poi_rs[1]
                # pos_rs.pose.position.z = poi_rs[2]
                # pos_iiwa = iiwa.tf_listener.transformPose('/iiwa_link_0', pos_rs)
                # rospy.loginfo("Object 3D position w.r.t. iiwa base from: {}".format(pos_iiwa.pose.position))
            # display image stream, press 'ESC' or 'q' to terminate
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
