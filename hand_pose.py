import os
import argparse
import matplotlib.pyplot as plt
import h5py

import numpy as np

cwd = os.getcwd()

parser = argparse.ArgumentParser()
parser.add_argument("--path", type=str, default='./data/test1', help="")
args = parser.parse_args()
#path = os.path.join(cwd, args.path)
path = args.path
print("WORKING ON PATH: ", path)
sensor_prefix = 'act'
target_prefix = 'des'
value_prefixes = {
    'Right hand position': ['rh_pos', 'rh_vel'],
    'Left hand position': ['lh_pos', 'lh_vel'],
    'Right hand orientation': ['rh_ori', 'rh_ori_vel'],
    'Left hand orientation': ['lh_ori', 'lh_ori_vel'],
}

joint_prefixes = {
    # 0: 'l_hip_ie',
    # 1: 'l_hip_aa',
    # 2: 'l_hip_fe',
    # 3: 'l_knee_fe_jp',
    # 4: 'l_knee_fe_jd',
    # 5: 'l_ankle_fe',
    # 6: 'l_ankle_ie',
    #LH
    7: 'l_shoulder_fe',
    8: 'l_shoulder_aa',
    9: 'l_shoulder_ie',
    10: 'l_elbow_fe',
    11: 'l_wrist_ps',
    12: 'l_wrist_pitch',
    #neck
    13: 'neck_pitch',
    #RF
    # 14: 'r_hip_ie',
    # 15: 'r_hip_aa',
    # 16: 'r_hip_fe',
    # 17: 'r_knee_fe_jp',
    # 18: 'r_knee_fe_jd',
    # 19: 'r_ankle_fe',
    # 20: 'r_ankle_ie',
    #RH
    # 21: 'r_shoulder_fe',
    # 22: 'r_shoulder_aa',
    # 23: 'r_shoulder_ie',
    # 24: 'r_elbow_fe',
    # 25: 'r_wrist_ps',
    # 26: 'r_wrist_pitch',
}

for root, dirs, files in os.walk(path, topdown=False):
    for name in files:
        if name.startswith("draco_controller_data") and name.endswith(".mat"):
            ctrl_data = h5py.File(os.path.join(root, name))
        if name.startswith("draco_icp_data") and name.endswith(".mat"):
            icp_data = h5py.File(os.path.join(root, name))
        if name.startswith("draco_state_estimator_data") and name.endswith(
                ".mat"):
            estimator_data = h5py.File(os.path.join(root, name))
        if name.startswith("draco_state_estimator_kf_data") and name.endswith(
                ".mat"):
            estimator_kf_data = h5py.File(os.path.join(root, name))
        # if name.endswith(".hdf5"):
        # joint_data = h5py.File(os.path.join(root, name))

# print(joint_data.keys())
initialized_idx = np.where(np.asarray(ctrl_data['state']) > 2.)[0][0]
initialized_idx = int(initialized_idx)

# for idx, value_prefix in joint_prefixes.items():
# fig, axes = plt.subplots(2)
# fig.suptitle(value_prefix, fontsize=16)

# axes[0].plot(np.array(joint_data['time']),
# np.array(joint_data['act_pos'])[:, idx],
# color='b')
# axes[0].plot(np.array(joint_data['time']),
# np.array(joint_data['des_pos'])[:, idx],
# color='r')
# axes[0].axvspan(0,
# ctrl_data['time'][initialized_idx],
# alpha=0.3,
# color='green')
# axes[0].set_ylabel('{} [rad]'.format(value_prefix))
# axes[0].set_xlim(left=0)

# axes[1].plot(np.array(joint_data['time']),
# np.array(joint_data['act_vel'])[:, idx],
# color='b')
# axes[1].plot(np.array(joint_data['time']),
# np.array(joint_data['des_vel'])[:, idx],
# color='r')
# axes[1].axvspan(0,
# ctrl_data['time'][initialized_idx],
# alpha=0.3,
# color='green')
# axes[1].set_ylabel('{} [rad/s]'.format(value_prefix))
# axes[1].set_xlim(left=0)

# axes[1].set_xlabel('time [sec]')

#plt.show()

# fig.set_figwidth(20)
# fig.set_figheight(10)
# fig.savefig(os.path.join(path, '{}.png'.format(value_prefix)))
#exit()
# >>> import numpy as np
# >>> import pickle
# >>> import h5py
# >>> name = "0222_012926.pkl"
# >>> f = open( name, 'rb')
# >>> joint_data = pickle.load(f, fix_imports=True)
# >>> dataset = h5py.File('joint_states.hdf5', 'w')
# >>> dataset.create_dataset('act_pos', data=np.array(joint_data['act_pos']), compression="gzip", chunks=True, dtype='f')
# <HDF5 dataset "act_pos": shape (40001, 27), type "<f4">
# >>> dataset.create_dataset('act_vel', data=np.array(joint_data['act_vel']), compression="gzip", chunks=True, dtype='f')
# <HDF5 dataset "act_vel": shape (40001, 27), type "<f4">
# >>> dataset.create_dataset('des_pos', data=np.array(joint_data['des_pos']), compression="gzip", chunks=True, dtype='f')
# <HDF5 dataset "des_pos": shape (40001, 27), type "<f4">
# >>> dataset.create_dataset('des_vel', data=np.array(joint_data['des_vel']), compression="gzip", chunks=True, dtype='f')
# <HDF5 dataset "des_vel": shape (40001, 27), type "<f4">
# >>> dataset.create_dataset('time', data=np.array(joint_data['time']), compression="gzip", chunks=True, dtype='f')
# <HDF5 dataset "time": shape (40001,), type "<f4">
# >>> dataset.close()
# >>> exit()

wbc_time = len(ctrl_data['des_lh_pos'])
time = ctrl_data['time'][len(ctrl_data['time']) - wbc_time:]

for topic, value_prefix in value_prefixes.items():
    fig, axes = plt.subplots(2, 3)
    fig.suptitle(topic, fontsize=16)

    if topic.endswith('position'):
        label = [['x', 'y', 'z'], [r'$\dot{x}$', r'$\dot{y}$', r'$\dot{z}$']]
        unit = ['m', 'm/s']
    else:
        label = [[r'${\phi}$', r'${\theta}$', r'${\psi}$'],
                 [r'$\dot{\phi}$', r'$\dot{\theta}$', r'$\dot{\psi}$']]
        unit = ['rad', 'rad/s']

    for idx in range(2):
        for subidx, coord in enumerate(label[idx]):
            axes[idx][subidx].plot(
                time,
                ctrl_data[f'{sensor_prefix}_{value_prefix[idx]}'][:, subidx],
                color='b')
            axes[idx][subidx].plot(
                time,
                ctrl_data[f'{target_prefix}_{value_prefix[idx]}'][:, subidx],
                color='r')
            axes[idx][subidx].axvspan(0,
                                      ctrl_data['time'][initialized_idx],
                                      alpha=0.3,
                                      color='green')
            axes[idx][subidx].set_ylabel('{} [{}]'.format(coord, unit[idx]))
            axes[idx][subidx].set_xlim(left=0)
            if idx == 1:
                axes[idx][subidx].set_xlabel('time [sec]')
    fig.set_figwidth(20)
    fig.set_figheight(10)
    fig.savefig(os.path.join(path, '{}.png'.format(topic)))

#plt.show()
