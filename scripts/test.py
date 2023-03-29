import numpy as np
import cv2
import h5py

f = h5py.File("steve_cap_1679536803.hdf5", "r")
for i in range(len(f['obs/rgb'])):
    img = f['obs/stereo'][i]
    img_ra = np.array(img, dtype=np.uint8)
    cv2.imshow('test', img_ra)
    cv2.waitKey(10)


