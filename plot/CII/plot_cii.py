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

cii = []

with open(args.file, 'rb') as file:
    while True:
        try:
            data = pickle.load(file)
            cii.append(data['cii'])
        except EOFError:
            break

draco_cii = np.array(cii)

fig, ax = plt.subplots()
ax.grid(True)
ax = sns.violinplot(data=draco_cii, color="lightgray", linewidth=3)
ax = sns.stripplot(data=draco_cii,
                   color='blue',
                   jitter=True,
                   alpha=0.8,
                   zorder=1)
# ax.set_xticklabels('draco')
ax.set_ylabel('CII')

plt.show()
