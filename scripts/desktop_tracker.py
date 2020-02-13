#!/usr/bin/env python
""" LBR iiwa track an object on desktop
    Depend on:
        roslaunch iiwa_tracker iiwa_gazebo.launch (tf defined)
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import sys
import os
import time

import cv2
import pyrealsense2 as rs
import numpy as np
from numpy import pi

import torch

from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker

import rospy
import tf
from robots.lbr import iiwaRobot
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from iiwa_msgs.msg import JointPosition


# iiwa's initial perching pose
JOINT_PERCH = JointPosition()
JOINT_PERCH.position.a1 = 0 #-pi/6
JOINT_PERCH.position.a2 = 0 #pi/3
JOINT_PERCH.position.a3 = 0 #pi/6
JOINT_PERCH.position.a4 = -pi/3 #-pi/2
JOINT_PERCH.position.a5 = 0 #-pi/6OINT_PERCH.position.a1 = -24.51/180*pi
JOINT_PERCH.position.a6 = pi/2 #-pi/4
JOINT_PERCH.position.a7 = -pi/6 #-pi/4
# JOINT_PERCH.position.a1 = 0 #-pi/6
# JOINT_PERCH.position.a2 = pi/6 #pi/3
# JOINT_PERCH.position.a3 = 0 #pi/6
# JOINT_PERCH.position.a4 = -pi/3 #-pi/2
# JOINT_PERCH.position.a5 = 0 #-pi/6OINT_PERCH.position.a1 = -24.51/180*pi
# JOINT_PERCH.position.a6 = pi/3
# JOINT_PERCH.position.a7 = -pi/6 #-pi/4
# define boundaries
X_MIN = 0.4
X_MAX = 1
Y_MIN = -.4
Y_MAX = .4
Z_MIN = .2
Z_MAX = 1.

def trig(angle):
    return np.cos(angle),np.sin(angle)

def transform(rotation, translation):
    Cx,Sx = trig(rotation[0])
    Cy,Sy = trig(rotation[1])
    Cz,Sz = trig(rotation[2])
    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]
    mat_trans = np.array([[1,0,0,dX],
                          [0,1,0,dY],
                          [0,0,1,dZ],
                          [0,0,0,1]])
    mat_rotX = np.array([[1,0,0,0],
                         [0,Cx,-Sx,0],
                         [0,Sx,Cx,0],
                         [0,0,0,1]])
    mat_rotY = np.array([[Cy,0,Sy,0],
                         [0,1,0,0],
                         [-Sy,0,Cy,0],
                         [0,0,0,1]])
    mat_rotZ = np.array([[Cz,-Sz,0,0],
                         [Sz,Cz,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
    return np.dot(mat_rotZ,np.dot(mat_rotY,np.dot(mat_rotX,mat_trans)))

def cartesian_to_matrix(cp):
    position = cp.position
    orientation = cp.orientation
    matrix = np.eye(4)
    # translation
    matrix[0,3] = position.x# in meter
    matrix[1,3] = position.y
    matrix[2,3] = position.z
    # quaternion to matrix
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    Nq = w*w + x*x + y*y + z*z
    if Nq < 0.001:
        return matrix

    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    matrix=np.array([[1.0-(yY+zZ), xY-wZ, xZ+wY, position.x],
            [xY+wZ, 1.0-(xX+zZ), yZ-wX, position.y],
            [xZ-wY, yZ+wX, 1.0-(xX+yY), position.z],
            [0, 0, 0, 1]])
    return matrix

# homougenous matrix to quaternion
def matrix_to_cartesian(mat):
    rot = np.array([mat[0,0:3],mat[1,0:3],mat[2,0:3]])
    trans = np.array([mat[0,3],mat[1,3],mat[2,3]])
    x = trans[0]
    y = trans[1]
    z = trans[2]
    qw = 0.5*np.sqrt(1+rot[0,0]+rot[1,1]+rot[2,2])
    qx = (rot[2,1]-rot[1,2])/(4*qw)
    qy = (rot[0,2]-rot[2,0])/(4*qw)
    qz = (rot[1,0]-rot[0,1])/(4*qw)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def main():
    # instantiate iiwa
    iiwa = iiwaRobot()
    time.sleep(4) # allow iiwa taking some time to wake up
    # zero joints
    iiwa.move_joint(commit=True)
    # iiwa get ready
    iiwa.move_joint(JOINT_PERCH, commit=True)
    time.sleep(4)
    rospy.loginfo("iiwa is ready")
    # read TCP orientation
    QUAT = Quaternion()
    QUAT.x = iiwa.cartesian_pose.orientation.x
    QUAT.y = iiwa.cartesian_pose.orientation.y
    QUAT.z = iiwa.cartesian_pose.orientation.z
    QUAT.w = iiwa.cartesian_pose.orientation.w
    # Configure realsense D435 depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    # Create an align object
    align_to = rs.stream.color
    align = rs.align(align_to)
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
    video_name = 'D435_color'
    cv2.namedWindow(video_name, cv2.WND_PROP_FULLSCREEN)
    first_frame = True
    while True:
        # wait image stream and select object of interest
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
        if first_frame:
            try:
                init_rect = cv2.selectROI(video_name, color_image, False, False)
            except:
                exit()
            tracker.init(color_image, init_rect)
            first_frame = False
        else:
            # start tracking
            outputs = tracker.track(color_image)
            polygon = np.array(outputs['polygon']).astype(np.int32)
            cv2.polylines(color_image, [polygon.reshape((-1, 1, 2))],
                          True, (0, 255, 0), 3)
            mask = ((outputs['mask'] > cfg.TRACK.MASK_THERSHOLD) * 255)
            mask = mask.astype(np.uint8)
            mask = np.stack([mask, mask*255, mask]).transpose(1, 2, 0)
            color_image = cv2.addWeighted(color_image, 0.77, mask, 0.23, -1)
            bbox = list(map(int, outputs['bbox']))
            poi_pixel = [int(bbox[0]+0.5*bbox[2]), int(bbox[1]+0.5*bbox[3])]
            poi_depth = depth_frame.get_distance(poi_pixel[0], poi_pixel[1])
            poi_rs = rs.rs2_deproject_pixel_to_point(depth_intrinsics, poi_pixel, poi_depth)
            print("Object 3D position w.r.t. camera frame: {}".format(poi_rs))
            if not np.allclose(poi_rs, np.zeros(3)):
                # compute transformed position of poi w.r.t. iiwa_link_0
                # transfrom = iiwa.tf_listener.getLatestCommonTime('/iiwa_link_0', '/rs_d435')
                # pos_rs = PoseStamped()
                pos_rs = Pose()
                # pos_rs.header.frame_id = 'rs_d435'
                pos_rs.orientation.x = iiwa.cartesian_pose.orientation.x
                pos_rs.orientation.y = iiwa.cartesian_pose.orientation.y
                pos_rs.orientation.z = iiwa.cartesian_pose.orientation.z
                pos_rs.orientation.w = iiwa.cartesian_pose.orientation.w
                pos_rs.position.x = poi_rs[0]
                pos_rs.position.y = poi_rs[1]
                pos_rs.position.z = poi_rs[2] - 0.2
                # tranform
                gripper_pose = cartesian_to_matrix(iiwa.cartesian_pose)
                print(gripper_pose)
                T_ge = transform([0,0,0],[0,0,0.16743])
                print(T_ge)
                ee_pose = np.dot(gripper_pose,np.linalg.inv(T_ge))
                print(ee_pose)
                T_c = transform([0.0, 0.0, -pi/3],[0.05*np.sin(pi/3), 0.05*np.cos(pi/3), 0.1])
                # T_c = transform([pi/6, 0.00, 0.00],[-0.05*np.sin(pi/6), 0.05*np.cos(pi/6), -0.1])
                print(T_c)
                target_pose = np.dot(ee_pose,np.dot(T_c, cartesian_to_matrix(pos_rs)))




                # pos_iiwa = iiwa.tf_listener.transformPose('/iiwa_link_0', pos_rs)
                # rospy.loginfo("Object 3D position w.r.t. iiwa base from: {}\n ee w.r.t. iiwa base: {}".format(pos_iiwa.pose.position, iiwa.cartesian_pose.position))
                # set cartesian goal
                iiwa.goal_carte_pose.header.frame_id = 'iiwa_link_0'
                t_pose=matrix_to_cartesian(target_pose)
                # iiwa.goal_carte_pose.pose = matrix_to_cartesian(target_pose)
                iiwa.goal_carte_pose.pose.position.x = np.clip(t_pose.position.x, X_MIN, X_MAX)
                iiwa.goal_carte_pose.pose.position.y = np.clip(t_pose.position.y, Y_MIN, Y_MAX)
                iiwa.goal_carte_pose.pose.position.z = np.clip(t_pose.position.z, Z_MIN, Z_MAX)
                iiwa.goal_carte_pose.pose.orientation = QUAT
                iiwa.move_cartesian(cartesian_pose=iiwa.goal_carte_pose)

        # display image stream, press 'ESC' or 'q' to terminate
        cv2.imshow(video_name, color_image)
        key = cv2.waitKey(40)
        if key in (27, ord("q")):
            break

    time.sleep(4)
    iiwa.move_joint(joint_position=JOINT_PERCH)
    time.sleep(4)
    pipeline.stop()
    rospy.loginfo("Finished")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
