"""A script to execute the neural network policy on the real robot
or saves the data for training. 

This script will listen to 2 zmq queues: one from the control PC and
the other one from the camera C++ script. It then syncs the data
and either saves them into an HDF5 file or executes the neural network
"""
from ..build.messages.draco_pb2 import *
import zmq
import sys
import os
import argparse
import collections

import numpy as np

cwd = os.getcwd()
sys.path.append(cwd + '/build')
sys.path.append(cwd)


parser = argparse.ArgumentParser()
parser.add_argument("--control_ip", type=str, default="")
parser.add_argument("--camera_ip", type=str, default="")
parser.add_argument("--save_data", type=bool, default=False)
args = parser.parse_args()
save_data = args.save_data

# ==========================================================================
# Socket initialize
# ==========================================================================
context = zmq.Context()
socket = context.socket(zmq.SUB)

socket.connect(args.control_ip)
socket.setsockopt_string(zmq.SUBSCRIBE, "")


# ==========================================================================
# Saving data for training
# ==========================================================================


if save_data:
    data_buffer = {
        'obs/joint_pos': collections.deque(),
        'obs/joint_vel': collections.deque(),
        'obs/local_lh_pos': collections.deque(),
        'obs/local_rh_pos': collections.deque(),
        'obs/local_lf_pos': collections.deque(),
        'obs/local_rf_pos':  collections.deque(),
        'obs/local_lh_quat': collections.deque(),
        'obs/local_rh_quat': collections.deque(),
        'obs/local_lf_quat': collections.deque(),
        'obs/local_rf_quat': collections.deque(),
        'obs/action/local_lh_pos': collections.deque(),
        'obs/action/local_lh_ori': collections.deque(),
        'obs/action/local_rh_pos': collections.deque(),
        'obs/action/local_rh_ori': collections.deque(),
        'obs/action/local_lf_pos': collections.deque(),
        'obs/action/local_lf_ori': collections.deque(),
        'obs/action/local_rf_pos': collections.deque(),
        'obs/action/local_rf_ori': collections.deque(),
        'obs/state': collections.deque(),
        'obs/action/l_gripper': collections.deque(),
        'obs/action/r_gripper': collections.deque(),
        'obs/rgb': collections.deque(),
        'obs/stereo': collections.deque(),
        'est_base_joint_pos': collections.deque(),
        'est_base_joint_ori': collections.deque(),
        'kf_base_joint_pos': collections.deque(),
        'kf_base_joint_ori': collections.deque(),
        'timestamp': collections.deque(),
    }

msg = pnc_msg()
while True:
    msg.ParseFromString(socket.recv())

    if save_data:
        # TODO: add velocity
        data_buffer['obs/joint'].append(np.concatenate(
            np.cos(msg.joint_positions), np.sin(msg.joint_positions)))
        data_buffer['est_base_joint_pos'].append(list(msg.est_base_joint_pos))
        data_buffer['est_base_joint_ori'].append(list(msg.est_base_joint_ori))
        data_buffer['kf_base_joint_pos'].append(list(msg.kf_base_joint_pos))
        data_buffer['kf_base_joint_ori'].append(list(msg.kf_base_joint_ori))
        data_buffer['timestamp'].append(msg.timestamp)
