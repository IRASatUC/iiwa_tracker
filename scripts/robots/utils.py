#!/usr/bin/env python
""" Helper utilities
"""
from __future__ import absolute_import, division, print_function

import sys
import time
import numpy as np

def jq_to_array(joint_quantity):
    """
    Convert JointQuantity into a numpy array
    Args:
        joint_quantity: iiwa_msgs.msg.JointQuantity
    Returns:
        j_arr: 1D array
    """
    assert joint_quantity._type == 'iiwa_msgs/JointQuantity'
    j_arr = np.array([
        joint_quantity.a1,
        joint_quantity.a2,
        joint_quantity.a3,
        joint_quantity.a4,
        joint_quantity.a5,
        joint_quantity.a6,
        joint_quantity.a7
    ])

    return j_arr
