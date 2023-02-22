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
value_prefix = {'Right hand position': ['rh_pos', 'rh_vel'],
                'Left hand position': ['lh_pos', 'lh_vel'],
                'Right hand orientation': ['rh_ori', 'rh_ori_vel'],
                'Left hand orientation': ['lh_ori', 'lh_ori_vel'],}

for root, dirs, files in os.walk(path, topdown=False):
    for name in files:
        if name.startswith("draco_controller_data") and name.endswith(".mat"):
            ctrl_data = h5py.File(os.path.join(root, name))
        if name.startswith("draco_icp_data") and name.endswith(".mat"):
            icp_data = h5py.File(os.path.join(root, name))
        if name.startswith("draco_state_estimator_data") and name.endswith(".mat"):
            estimator_data = h5py.File(os.path.join(root, name))
        if name.startswith("draco_state_estimator_kf_data") and name.endswith(".mat"):
            estimator_kf_data = h5py.File(os.path.join(root, name))

initialized_idx = np.where(np.asarray(ctrl_data['state']) > 2.)[0][0]
initialized_idx = int(initialized_idx)

for topic, value_prefix in value_prefix.items():
    fig, axes = plt.subplots(2, 3)
    fig.suptitle(topic, fontsize=16)

    if topic.endswith('position'):
        label = [['x', 'y', 'z'], [r'$\dot{x}$', r'$\dot{y}$', r'$\dot{z}$']]
        unit = ['m', 'm/s']
    else:
        label = [[r'${\phi}$', r'${\theta}$', r'${\psi}$'], [r'$\dot{\phi}$', r'$\dot{\theta}$', r'$\dot{\psi}$']]
        unit = ['rad', 'rad/s']

    for idx in range(2):
        for subidx, coord in enumerate(label[idx]):
            axes[idx][subidx].plot(ctrl_data['time'], ctrl_data[f'{sensor_prefix}_{value_prefix[idx]}'][:, subidx], color='b')
            axes[idx][subidx].plot(ctrl_data['time'], ctrl_data[f'{target_prefix}_{value_prefix[idx]}'][:, subidx], color='r')
            axes[idx][subidx].axvspan(0, ctrl_data['time'][initialized_idx], alpha=0.3, color='green')
            axes[idx][subidx].set_ylabel('{} {}'.format(coord, unit[idx]))
            axes[idx][subidx].set_xlim(left=0)
            if idx == 1:
                axes[idx][subidx].set_xlabel('time [sec]')
    fig.set_figwidth(20)
    fig.set_figheight(10)
    fig.savefig(os.path.join(path,'{}.png'.format(topic)))

#plt.show()
