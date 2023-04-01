import numpy as np
import cv2
import h5py

f = h5py.File("./test_data_set/steve_box_01-04-2023--02:17:29.hdf5", "r")
print('obs/act_global_lh_pos', f['obs/act_global_lh_pos'][30].astype(np.float32))
print('action/local_rh_pos', f['action/local_rh_pos'][20])
print('action/local_lh_pos', f['action/local_lh_pos'][20])
print('action/local_rh_ori', f['action/local_rh_ori'][20])
print('action/local_lh_ori', f['action/local_lh_ori'][20])

f = f['data/demo_0']
print(f['obs/rgb'].shape)
print(f['obs/rgb'][20])
print(f['obs/stereo'][20])
print('obs/act_local_lh_pos', f['obs/act_local_lh_pos'][20])
print('obs/act_local_rh_pos', f['obs/act_local_rh_pos'][20])
print('obs/act_local_lf_pos', f['obs/act_local_lf_pos'][20])
print('obs/act_local_rf_pos', f['obs/act_local_rf_pos'][20])
print('obs/act_local_lh_ori', f['obs/act_local_lh_ori'][20])
print('obs/act_local_rh_ori', f['obs/act_local_rh_ori'][20])
print('obs/act_local_lf_ori', f['obs/act_local_lf_ori'][20])
print('obs/act_local_rf_ori', f['obs/act_local_rf_ori'][20])

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
