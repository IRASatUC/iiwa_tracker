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

import rospy
import pdb


def main():
    # Configure realsense D435 depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)
    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1.8 #1.8 meter
    clipping_distance = clipping_distance_in_meters / depth_scale
    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    video_name = 'realsense_D435'
    cv2.namedWindow(video_name, cv2.WND_PROP_FULLSCREEN)
    # begin tracking
    while True:
        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
        # convert image to numpy arrays
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            # # Remove background - Set pixels further than clipping_distance to grey
            # grey_color = 153
            # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            poi_pixel = [320,320]
            poi_depth = depth_frame.get_distance(poi_pixel[0], poi_pixel[1])
            if poi_depth >= 1 or poi_depth <= 0
            print("poi_depth: {}".format(poi_depth))
            poi_rs = rs.rs2_deproject_pixel_to_point(depth_intrinsics, poi_pixel, poi_depth)
            print("poi_rs: {}".format(poi_rs))
            pdb.set_trace()
            # display image stream, press 'ESC' or 'q' to terminate
            cv2.imshow(video_name, color_image)
            key = cv2.waitKey(40)
            if key in (27, ord("q")):
                break

    pipeline.stop()

if __name__ == '__main__':
    main()
