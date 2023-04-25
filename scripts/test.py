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
for j in range(0, f['obs/rgb'].shape[0]):
    img = f['obs/rgb'][j]
    cv2.imshow('test', img)
    cv2.waitKey(20)
print('action/local_lh_pos', f['action/local_lh_pos'][40])
print('obs/act_global_lh_pos', f['obs/act_global_lh_pos'][40])
print('obs/des_global_lh_pos', f['obs/des_global_lh_pos'][40])

print('action/local_rh_pos', f['action/local_rh_pos'][40])
print('obs/act_global_rh_pos', f['obs/act_global_rh_pos'][40])

print('action/local_lh_ori', f['action/local_lh_ori'][40])
print('obs/act_global_lh_ori', f['obs/act_global_lh_ori'][40])

print('action/local_rh_ori', f['action/local_rh_ori'][40])
print('obs/act_global_rh_ori', f['obs/act_global_rh_ori'][40])

print('global_base_pos', f['global_base_pos'][40])
print('global_base_ori', f['global_base_ori'][40])


f = f['data/demo_0']
print(f['obs/rgb'].shape)
print(f['obs/rgb'][20])
print(f['obs/stereo'][20])
print('obs/act_local_lh_pos', f['obs/act_local_lh_pos'][10])
print('obs/act_local_rh_pos', f['obs/act_local_rh_pos'][10])
print('obs/act_local_lf_pos', f['obs/act_local_lf_pos'][10])
print('obs/act_local_rf_pos', f['obs/act_local_rf_pos'][10])
print('obs/act_local_lh_ori', f['obs/act_local_lh_ori'][10])
print('obs/act_local_rh_ori', f['obs/act_local_rh_ori'][10])
print('obs/act_local_lf_ori', f['obs/act_local_lf_ori'][10])
print('obs/act_local_rf_ori', f['obs/act_local_rf_ori'][10])

print('actions', f['actions'][20])
print(f['obs/joint_pos'][3])
print(f['obs/joint_vel'][3])

for i in range(len(f['obs/stereo'])):
    img = f['obs/rgb'][i]
    img_ra = np.array(img, dtype=np.uint8)
    cv2.imshow('test', img)
    cv2.waitKey(100)


# for i in range(len(f['obs/rgb'])):
#    img = f['obs/stereo'][i]
#    img_ra = np.array(img, dtype=np.uint8)
#    cv2.imshow('test', img_ra)
#    cv2.waitKey(10)
