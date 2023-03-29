import numpy as np
import cv2
import h5py

f = h5py.File("../../datasets/real_robot/steve_cap_1679536373.hdf5", "r")
print(f['obs/rgb'][0].shape)
print(f['obs/rgb'][0])
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
