import numpy as np
import cv2
import h5py

f = h5py.File("../steve_box_30-03-2023--19:20:36.hdf5", "r")
print(f['obs/rgb'].shape)
print(f['obs/rgb'][1])
print(f['obs/stereo'][1])
print(f['obs/local_lh_pos'][39])
print(f['obs/local_rh_pos'][13])
print(f['obs/local_lf_pos'][100])
print(f['obs/local_rf_pos'][100])
print(np.max(f['obs/local_lh_ori'], axis=0))
print(f['obs/local_rh_ori'][100])
print(f['obs/local_lf_ori'][8])
print(f['obs/local_rf_ori'][3])
print(f['action/local_rh_pos'][3])
print(f['action/local_lh_pos'][3])
print(f['action/local_rh_ori'][3])
print(f['action/local_lh_ori'][3])
print(f['obs/joint_pos'][3])
print(f['obs/joint_vel'][3])


for i in range(len(f['obs/rgb'])):
    img = f['obs/stereo'][i]
    img_ra = np.array(img, dtype=np.uint8)
    cv2.imshow('test', img)
    cv2.waitKey(10)
# for i in range(len(f['obs/rgb'])):
#    img = f['obs/stereo'][i]
#    img_ra = np.array(img, dtype=np.uint8)
#    cv2.imshow('test', img_ra)
#    cv2.waitKey(10)
