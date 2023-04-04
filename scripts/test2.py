import argparse
import numpy as np
import cv2
import h5py
from scipy.spatial.transform import Rotation as R

# take in the first argument as the file path
parser = argparse.ArgumentParser()
parser.add_argument("file_path", help="path to the hdf5 file")
args = parser.parse_args()

f = h5py.File(args.file_path, "r")
i = 0
while (f['vr_ready'][i] == False):
    i += 1

print(f['vr_ready'][i])
print(f['vr_ready'][i+ 10])
print(i)
print('act_rh_pos', f['act_rh_pos'][i])
print('act_lh_pos', f['act_lh_pos'][i])
print('act_rh_ori', f['act_rh_ori'][i])
print('act_lh_ori', f['act_lh_ori'][i])
print('action_local_lh_pos', f['action_local_lh_pos'][i])
print('action_local_rh_pos', f['action_local_rh_pos'][i])
print('action_local_lh_ori', f['action_local_lh_ori'][i])
print('action_local_rh_ori', f['action_local_rh_ori'][i])
