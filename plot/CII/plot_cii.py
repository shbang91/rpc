import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

import pickle

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str)
args = parser.parse_args()



r_hip_aa_jp = []
r_hip_fe_jp = []
cii = []

with open(args.file,'rb') as file:
    while True:
        try:
            data = pickle.load(file)
            r_hip_aa_jp.append(data['r_hip_aa'])
            r_hip_fe_jp.append(data['r_hip_fe'])
            cii.append(data['cii'])
        except EOFError:
            break
def adjacent_values(vals, q1, q3):
    upper_adjacent_value = q3 + (q3 - q1) * 1.5
    upper_adjacent_value = np.clip(upper_adjacent_value, q3, vals[-1])

    lower_adjacent_value = q1 - (q3 - q1) * 1.5
    lower_adjacent_value = np.clip(lower_adjacent_value, vals[0], q1)
    return lower_adjacent_value, upper_adjacent_value


def set_axis_style(ax, labels):
    ax.xaxis.set_tick_params(direction='out')
    ax.xaxis.set_ticks_position('bottom')
    ax.set_xticks(np.arange(1, len(labels) + 1), labels=labels)
    ax.set_xlim(0.25, len(labels) + 0.75)
    ax.set_xlabel('Sample name')

# print(r_hip_aa_jp)
# print(r_hip_fe_jp)
# print(cii)

x = np.array(r_hip_aa_jp)
y = np.array(r_hip_fe_jp)
z = np.array(cii)

# fig, ax = plt.subplots()
# ax.set_title('draco_cii')
# vp = ax.violinplot(dataset=z, showmeans=True, showmedians=False, showextrema=True)

# quartile1, medians, quartile3 = np.percentile(z, [25,50,75])
# whiskers = np.array([adjacent_values(sorted_array, q1, q3) for sorted_array, q1, q3 in zip(cii, quartile1, quartile3)])
# whiskers_min, whiskers_max = whiskers[:, 0], whiskers[:,1]

# inds = np.arange(1, len(medians)+1)
# ax.scatter(medians, marker='o', color='white', s=30, zorder=3)
# ax.vlines(quartile1, quartile3, color='k', linestyle='-', lw=5)
# ax.vlines(inds, whiskers_min, whiskers_max, color='k', linestyle='-', lw=1)

# labels = ['Draco']
# labels = ['Draco', 'Atlas', 'Digit']
# for ax in [ax]:
    # set_axis_style(ax, labels)

# sp = ax.stripplot(data=z)

ax = sns.violinplot(data=z, color=".9")
ax = sns.stripplot(data=z)

plt.show()
