"""
Reads the .mat files collected from the robot control PC and the folder
containing images from the robot camera and creates an HDF5 dataset. 
"""

import argparse
import os
import h5py
import numpy as np
import cv2

parser = argparse.ArgumentParser()
parser.add_argument(
    "--mat", "-m", help="the directory containing the .mat files")
parser.add_argument(
    "--images", "-i", help="the directory containing the images")
parser.add_argument(
    "--output", "-o", help="the output HDF5 file", default="dataset.hdf5")
args = parser.parse_args()

img_width = 800
img_height = 200

sensor_prefix = 'act'
target_prefix = 'des'
value_prefixes = {
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

OBS_JOINT_KEYS = [
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
joint_prefixes = {
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


# open the .mat files
ctrl_data = None
estimator_data = None
for root, dirs, files in os.walk(args.mat, topdown=False):
    for name in files:
        if name.startswith("draco_controller_data") and name.endswith(".mat"):
            ctrl_data = h5py.File(os.path.join(root, name))
        if name.startswith("draco_state_estimator_data") and name.endswith(
                ".mat"):
            estimator_data = h5py.File(os.path.join(root, name))
assert ctrl_data is not None
assert estimator_data is not None

output_file = h5py.File(args.output, "w")
output_data = output_file.create_group("data")
ep_group = output_data.create_group("demo_")
obs_group = ep_group.create_group("obs")

# format the observations
obs_joint_pos = np.column_stack(
    [estimator_data['joint_pos_act'][:, joint_prefixes[key]] for key in OBS_JOINT_KEYS])
obs_joint_vel = np.column_stack(
    [estimator_data['joint_pos_act'][:, joint_prefixes[key]] for key in OBS_JOINT_KEYS])
obs_group.create_dataset("joint", data=np.concatenate((np.cos(obs_joint_pos), np.sin(
    obs_joint_pos), obs_joint_vel), axis=1), compression="gzip", chunks=True, dtype="f")

# get images oberservations
image_timestamps = [int(os.path.splitext(filename)[0])
                    for filename in os.listdir(args.images)]
print(len(image_timestamps))

print("image timestamps", image_timestamps)
image_timestamps.sort()
print("image timestamps sorted", image_timestamps)
data_timestamps = ctrl_data['timestamp']
print("data timestamps", data_timestamps)
image_indices_to_keep = np.searchsorted(image_timestamps, data_timestamps)
obs_images = np.empty(
    (len(image_indices_to_keep), img_height, img_width, 1), dtype="uint8")
for idx in image_indices_to_keep:
    img = cv2.imread(os.path.join(
        args.images, str(image_timestamps[idx]) + ".png"))
    # covert cv2 image to numpy array
    obs_images[idx] = img
