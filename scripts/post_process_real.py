"""
Batch and post-process the datasets collected using nn_inference_and_collection.py
into a single dataset used to train the neural network.
"""

import argparse
import os
import h5py
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser()
# take in the path to the directory containing the hdf5 files as a positional argument
parser.add_argument(
    "path", help="the path to the directory containing the hdf5 files")
parser.add_argument(
    "--output", "-o", help="the output HDF5 file", default="dataset.hdf5")
args = parser.parse_args()

output_file = h5py.File(args.output, "w")
output_data = output_file.create_group("data")

demo_count = 0  # total number of episodes
total = 0  # total number of steps

# open the hdf5 files
for root, dirs, files in os.walk(args.path, topdown=False):
    for name in files:
        if name.endswith(".hdf5"):
            with h5py.File(os.path.join(root, name)) as demo_file:

                # dummy data since we aren't using RL?
                done = np.zeros(
                    demo_file['obs/joint_pos'].shape[0], dtype='uint64')

                # all data for this episode goes in this group
                ep_group = output_data.create_group(f"demo_{demo_count}")

                obs_group = ep_group.create_group("obs")
                # cos/sin encoding of joint pos concat with joint vel
                obs_group.create_dataset("joint", data=np.concatenate((np.cos(demo_file['obs/joint_pos']), np.sin(
                    demo_file['obs/joint_pos']), demo_file['obs/joint_vel']), axis=1), compression="gzip", chunks=True, dtype="f")
                for key in demo_file['obs'].keys():
                    # don't need joint pos/vel since we already have joint, and state doesn't matter for fixed base
                    if key not in ["joint_pos", "joint_vel", "state"]:
                        demo_file.copy(demo_file[f"obs/{key}"], obs_group, key)
                # TODO: flatten video?

                # flatten actions
                act_discrete = np.column_stack(
                    [demo_file['action/l_gripper'], demo_file['action/r_gripper']])

                # compute delta pos for trajectory
                act_trajecory_right_pos = demo_file['action/local_rh_pos']
                act_trajecory_left_pos = demo_file['action/local_lh_pos']
                act_trajecory_right_quat = demo_file['action/local_rh_ori']
                act_trajecory_left_quat = demo_file['action/local_lh_ori']

                act_trajecory_right_delta_pos = np.copy(
                    act_trajecory_right_pos)
                act_trajecory_right_delta_pos[1:
                                              ] -= act_trajecory_right_pos[:-1]
                act_trajecory_right_delta_pos[0] -= act_trajecory_right_pos[0]
                act_trajecory_left_delta_pos = np.copy(act_trajecory_left_pos)
                act_trajecory_left_delta_pos[1:] -= act_trajecory_left_pos[:-1]
                act_trajecory_left_delta_pos[0] -= act_trajecory_left_pos[0]

                act_trajecory_right_rot = R.from_quat(
                    act_trajecory_right_quat).as_matrix()
                act_trajecory_right_delta_rot = np.copy(
                    act_trajecory_right_rot)
                act_trajecory_right_delta_rot[1:] = act_trajecory_right_delta_rot[1:] @ (
                    act_trajecory_right_rot[:-1].transpose(0, 2, 1))
                act_trajecory_right_delta_rot[0] = act_trajecory_right_delta_rot[0] @ (
                    act_trajecory_right_rot[0].transpose())
                act_trajecory_right_delta_quat = R.from_matrix(
                    act_trajecory_right_delta_rot).as_quat()
                act_trajecory_left_rot = R.from_quat(
                    act_trajecory_left_quat).as_matrix()
                act_trajecory_left_delta_rot = np.copy(act_trajecory_left_rot)
                act_trajecory_left_delta_rot[1:] = act_trajecory_left_delta_rot[1:] @ (
                    act_trajecory_left_rot[:-1].transpose(0, 2, 1))
                act_trajecory_left_delta_rot[0] = act_trajecory_left_delta_rot[0] @ (
                    act_trajecory_left_rot[0].transpose())
                act_trajecory_left_delta_quat = R.from_matrix(
                    act_trajecory_left_delta_rot).as_quat()
                act_trajecory = np.column_stack(
                    [act_trajecory_right_delta_pos, act_trajecory_left_delta_pos, act_trajecory_right_delta_quat, act_trajecory_left_delta_quat])

                act_concat = np.column_stack([act_discrete, act_trajecory])

                ep_group.create_dataset("actions", data=act_concat)
                ep_group.create_dataset("dones", data=done, dtype='uint64')
                ep_group.create_dataset("rewards", data=done)
                ep_group.attrs["num_samples"] = int(done.shape[0])
                total += int(done.shape[0])

output_file.attrs["total"] = total
metadata = ""
output_file.attrs["env_args"] = metadata
output_file.close()


""" 
# get images oberservations
image_timestamps=[int(os.path.splitext(filename)[0])
                    for filename in os.listdir(args.images)]
print(len(image_timestamps))

print("image timestamps", image_timestamps)
image_timestamps.sort()
print("image timestamps sorted", image_timestamps)
data_timestamps = ctrl_data['timestamp']
print("data timestamps", data_timestamps)
image_indices_to_keep = np.searchsorted(image_timestamps, data_timestamps)
obs_images = np.empty(
    (len(image_indices_to_keep), img_height, img_width, 1), dtype = "uint8")
for idx in image_indices_to_keep:
    img=cv2.imread(os.path.join(
        args.images, str(image_timestamps[idx]) + ".png"))
    # covert cv2 image to numpy array
    obs_images[idx]=img


img_width=800
img_height=200

sensor_prefix='act'
target_prefix='des'
value_prefixes={
    'Right hand position': ['rh_pos', 'rh_vel'],
    'Left hand position': ['lh_pos', 'lh_vel'],
    'Right hand orientation': ['rh_ori', 'rh_ori_vel'],
    'Left hand orientation': ['lh_ori', 'lh_ori_vel'],
}


OBS_EEF_KEYS = [
    'rh_eef_pos', 'lh_eef_pos',
    'rf_foot_pos', 'lf_foot_pos',
    'rh_eef_quat', 'lh_eef_quat',
    'rf_foot_quat', 'lf_foot_quat',
]

OBS_JOINT_KEYS= [
    'r_hip_ie', 'r_hip_aa', 'r_hip_fe',
    'r_knee_fe_jp', 'r_knee_fe_jd', 'r_ankle_fe', 'r_ankle_ie',
    'l_hip_ie', 'l_hip_aa', 'l_hip_fe',
    'l_knee_fe_jp', 'l_knee_fe_jd', 'l_ankle_fe', 'l_ankle_ie',
    'r_shoulder_fe', 'r_shoulder_aa', 'r_shoulder_ie',
    'r_elbow_fe', 'r_wrist_ps', 'r_wrist_pitch',
    'l_shoulder_fe', 'l_shoulder_aa', 'l_shoulder_ie',
    'l_elbow_fe', 'l_wrist_ps', 'l_wrist_pitch',
    'neck_pitch',
]
joint_prefixes= {
    'l_hip_ie':           0,
    'l_hip_aa':           1,
    'l_hip_fe':           2,
    'l_knee_fe_jp':       3,
    'l_knee_fe_jd':       4,
    'l_ankle_fe':         5,
    'l_ankle_ie':         6,
    'l_shoulder_fe':      7,
    'l_shoulder_aa':      8,
    'l_shoulder_ie':      9,
    'l_elbow_fe':        10,
    'l_wrist_ps':        11,
    'l_wrist_pitch':     12,
    'neck_pitch':        13,
    'r_hip_ie':          14,
    'r_hip_aa':          15,
    'r_hip_fe':          16,
    'r_knee_fe_jp':      17,
    'r_knee_fe_jd':      18,
    'r_ankle_fe':        19,
    'r_ankle_ie':        20,
    'r_shoulder_fe':     21,
    'r_shoulder_aa':     22,
    'r_shoulder_ie':     23,
    'r_elbow_fe':        24,
    'r_wrist_ps':        25,
    'r_wrist_pitch':     26,
}
"""
