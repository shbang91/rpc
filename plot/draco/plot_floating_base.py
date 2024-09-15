import os
import sys
import pickle
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from plot.helper import plot_vector_traj

cwd = os.getcwd()
sys.path.append(cwd)
matplotlib.use("TkAgg")

st_idx = 10

# read pkl data & save the data in containers
time = []

# base_com_pos = []
# base_com_ori = []
# base_com_lin_vel = []
# base_com_ang_vel = []

base_joint_pos = []
base_joint_ori = []
base_joint_lin_vel = []
base_joint_ang_vel = []

est_base_joint_pos = []
est_base_joint_ori = []
est_base_joint_lin_vel = []
est_base_joint_ang_vel = []

with open("experiment_data/pnc.pkl", "rb") as file:
    while True:
        try:
            data = pickle.load(file)
            time.append(data["time"])

            base_joint_pos.append(data["base_joint_pos"])
            base_joint_ori.append(data["base_joint_ori"])
            base_joint_lin_vel.append(data["base_joint_lin_vel"])
            base_joint_ang_vel.append(data["base_joint_ang_vel"])

            est_base_joint_pos.append(data["est_base_joint_pos"])
            est_base_joint_ori.append(data["est_base_joint_ori"])
            est_base_joint_lin_vel.append(data["est_base_joint_lin_vel"])
            est_base_joint_ang_vel.append(data["est_base_joint_ang_vel"])
        except EOFError:
            break

time = np.array(time)[st_idx:]

base_joint_pos = np.stack(base_joint_pos, axis=0)[st_idx:, :]
base_joint_ori = np.stack(base_joint_ori, axis=0)[st_idx:, :]
base_joint_lin_vel = np.stack(base_joint_lin_vel, axis=0)[st_idx:, :]
base_joint_ang_vel = np.stack(base_joint_ang_vel, axis=0)[st_idx:, :]

est_base_joint_pos = np.stack(est_base_joint_pos, axis=0)[st_idx:, :]
est_base_joint_ori = np.stack(est_base_joint_ori, axis=0)[st_idx:, :]
est_base_joint_lin_vel = np.stack(est_base_joint_lin_vel, axis=0)[st_idx:, :]
est_base_joint_ang_vel = np.stack(est_base_joint_ang_vel, axis=0)[st_idx:, :]

axes = plot_vector_traj(
    time, base_joint_pos, None, ["x", "y", "z"], "g", "base_joint_pos"
)
plot_vector_traj(time, est_base_joint_pos, None, ["x", "y", "z"], "k", None, axes)

axes = plot_vector_traj(
    time, base_joint_ori, None, ["x", "y", "z", "w"], "g", "base_joint_ori"
)
plot_vector_traj(time, est_base_joint_ori, None, ["x", "y", "z", "w"], "k", None, axes)

axes = plot_vector_traj(
    time, base_joint_lin_vel, None, ["x", "y", "z"], "g", "base_joint_lin_vel"
)
plot_vector_traj(time, est_base_joint_lin_vel, None, ["x", "y", "z"], "k", None, axes)

axes = plot_vector_traj(
    time, base_joint_ang_vel, None, ["x", "y", "z"], "g", "base_joint_ang_vel"
)
plot_vector_traj(time, est_base_joint_ang_vel, None, ["x", "y", "z"], "k", None, axes)
plt.show()
