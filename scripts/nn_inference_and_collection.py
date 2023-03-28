"""A script to execute the neural network policy on the real robot
or saves the data for training. 

This script will listen to 2 zmq queues: one from the control PC and
the other one from the camera C++ script (which has 2 sockets for the 
rgb images and stereo grayscale images). It then syncs the data
and either saves them into an HDF5 file or executes the neural network
"""
import zmq
import sys
import os
import time

cwd = os.getcwd()
sys.path.append(cwd + '/build')
sys.path.append(cwd)

import argparse
import collections
import h5py
from messages.draco_pb2 import *

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--control_ip", type=str, default="tcp://192.168.1.184:5557")
parser.add_argument("--rgb_camera_ip", type=str, default="tcp://localhost:5557")
parser.add_argument("--stereo_camera_ip", type=str, default="tcp://localhost:5558")
parser.add_argument("--save_data", type=bool, default=True)
parser.add_argument("--demonstrator", type=str, default="steve")
parser.add_argument("--task", type=str, default="screwdriver")
args = parser.parse_args()
save_data = args.save_data

# ==========================================================================
# Socket initialize
# ==========================================================================
context = zmq.Context()
control_socket = context.socket(zmq.SUB)

control_socket.connect(args.control_ip)
control_socket.setsockopt_string(zmq.SUBSCRIBE, "")

rgb_streaming_socket = context.socket(zmq.PULL)
rgb_streaming_socket.set(zmq.CONFLATE, 1)
rgb_streaming_socket.connect(args.rgb_camera_ip)

stereo_streaming_socket = context.socket(zmq.PULL)
stereo_streaming_socket.set(zmq.CONFLATE, 1)
stereo_streaming_socket.connect(args.stereo_camera_ip)

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
        'obs/local_lh_ori': collections.deque(),
        'obs/local_rh_ori': collections.deque(),
        'obs/local_lf_ori': collections.deque(),
        'obs/local_rf_ori': collections.deque(),
        'obs/state': collections.deque(),
        'obs/rgb': collections.deque(),
        'obs/stereo': collections.deque(),
        'action/l_gripper': collections.deque(),
        'action/r_gripper': collections.deque(),
        'action/local_lh_pos': collections.deque(),
        'action/local_lh_ori': collections.deque(),
        'action/local_rh_pos': collections.deque(),
        'action/local_rh_ori': collections.deque(),
        'est_base_joint_pos': collections.deque(),
        'est_base_joint_ori': collections.deque(),
        'kf_base_joint_pos': collections.deque(),
        'kf_base_joint_ori': collections.deque(),
        'timestamp': collections.deque(),
    }

msg = pnc_msg()

if save_data:
    vr_ready_prev = False

prev_time = 0
new_time = 1
while True:
    new_time = time.time()
    fps = 1/(new_time-prev_time)
    print("fps: ", fps)
    prev_time = new_time

    msg.ParseFromString(control_socket.recv())
    print(msg.vr_ready)

    if save_data and msg.vr_ready:
        # TODO: add velocity
        data_buffer['obs/joint_pos'].append(list(msg.joint_positions))
        data_buffer['obs/joint_vel'].append(list(msg.joint_velocities))
        data_buffer['obs/local_lh_pos'].append(list(msg.local_lh_pos))
        data_buffer['obs/local_rh_pos'].append(list(msg.local_rh_pos))
        data_buffer['obs/local_lf_pos'].append(list(msg.local_lf_pos))
        data_buffer['obs/local_rf_pos'].append(list(msg.local_rf_pos))
        data_buffer['obs/local_lh_ori'].append(list(msg.local_lh_ori))
        data_buffer['obs/local_rh_ori'].append(list(msg.local_rh_ori))
        data_buffer['obs/local_lf_ori'].append(list(msg.local_lf_ori))
        data_buffer['obs/local_rf_ori'].append(list(msg.local_rf_ori))
        data_buffer['obs/state'].append(msg.state)
        data_buffer['action/l_gripper'].append(msg.l_gripper)
        data_buffer['action/r_gripper'].append(msg.r_gripper)
        data_buffer['action/local_lh_pos'].append(
            list(msg.action_local_lh_pos))
        data_buffer['action/local_lh_ori'].append(
            list(msg.action_local_lh_ori))
        data_buffer['action/local_rh_pos'].append(
            list(msg.action_local_rh_pos))
        data_buffer['action/local_rh_ori'].append(
            list(msg.action_local_rh_ori))
        data_buffer['est_base_joint_pos'].append(list(msg.est_base_joint_pos))
        data_buffer['est_base_joint_ori'].append(list(msg.est_base_joint_ori))
        data_buffer['kf_base_joint_pos'].append(list(msg.kf_base_joint_pos))
        data_buffer['kf_base_joint_ori'].append(list(msg.kf_base_joint_ori))
        data_buffer['timestamp'].append(msg.timestamp)

    rgb_img = rgb_streaming_socket.recv()
    stereo_img = stereo_streaming_socket.recv()
    if save_data and msg.vr_ready:
        data_buffer['obs/rgb'].append(np.frombuffer(rgb_img, dtype=np.uint8).reshape(200, 400, 3))

        data_buffer['obs/stereo'].append(np.frombuffer(stereo_img, dtype=np.uint8).reshape(200, 800, 1))
    if not msg.vr_ready and vr_ready_prev:
        print("Saving data...")
        # save data_buffer to hdf5 file
        with h5py.File(f"{args.demonstrator}_{args.task}_{int(time.time())}.hdf5", 'w') as f:
            for key, value in data_buffer.items():
                f.create_dataset(key, data=np.array(value))
        print("Done!")
    vr_ready_prev = msg.vr_ready
