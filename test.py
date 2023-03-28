import numpy as np
import cv2
import h5py

"""
f = h5py.File("steve_cap_1679536803.hdf5", "r")
for i in range(len(f['obs/rgb'])):
    img = f['obs/stereo'][i]
    img_ra = np.array(img, dtype=np.uint8)
    cv2.imshow('test', img_ra)
    cv2.waitKey(10)
"""


f = h5py.File("../datasets/real_robot/steve_box_1679537090.hdf5", "r")
print(f['action/local_lh_ori'][5])
print(f['obs/local_rh_ori'][5])
