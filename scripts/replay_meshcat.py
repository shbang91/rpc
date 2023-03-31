"""Replays robot joint data using Meshcat
"""

import time
import meshcat
import sys
import os
import numpy as np
import h5py
from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin

cwd = os.getcwd()
sys.path.append(cwd)

# take a command line argument for the hdf5 file to replay
if len(sys.argv) < 2:
    print("Usage: python replay.py <hdf5 file>")
    sys.exit(0)
hdf5_file = sys.argv[1]


model, collision_model, visual_model = pin.buildModelsFromUrdf(
    cwd + "/robot_model/draco/draco3_big_feet.urdf",
    cwd + "/robot_model/draco", pin.JointModelFreeFlyer())
viz = MeshcatVisualizer(model, collision_model, visual_model)
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
viz.loadViewerModel(rootNodeName="draco3")

# load the hdf5 file
with h5py.File(hdf5_file, "r") as f:
    # combine kf_base_joint_pos and joint_positions
    joint_positions = np.column_stack((f["kf_base_joint_pos"],
                                       f["kf_base_joint_ori"], f["obs/joint_pos"]))

for i in range(len(joint_positions)):
    viz.display(joint_positions[i])
    time.sleep(.05)
