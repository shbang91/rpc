import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

import pickle
import ipdb

cii_file_list = [
    cwd + '/experiment_data/draco_cii.pkl',
    cwd + '/experiment_data/draco_cii_collocated.pkl',
    cwd + '/experiment_data/atlas_cii.pkl',
    cwd + '/experiment_data/valkyrie_cii.pkl',
    cwd + '/experiment_data/cassie_cii.pkl'
]

draco_cii = []
draco_cii_collocated = []
atlas_cii = []
valkyrie_cii = []
cassie_cii = []

for cii_file in cii_file_list:
    with open(cii_file, 'rb') as file:
        while True:
            try:
                data = pickle.load(file)
                if cii_file == cwd + '/experiment_data/draco_cii.pkl':
                    draco_cii.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/draco_cii_collocated.pkl':
                    draco_cii_collocated.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/atlas_cii.pkl':
                    atlas_cii.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/valkyrie_cii.pkl':
                    valkyrie_cii.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/cassie_cii.pkl':
                    cassie_cii.append(data['cii'])
            except EOFError:
                break

pd_data = {
    'draco (proximal actuation)': draco_cii,
    'draco (collocated actuation)': draco_cii_collocated
    # 'atlas': atlas_cii,
    # 'valkyrie': valkyrie_cii
    # 'cassie': cassie_cii
}

df = pd.DataFrame(pd_data)

fig, ax = plt.subplots()
ax.grid(True)
ax = sns.violinplot(data=df, color="lightgray", linewidth=3)
# ax = sns.stripplot(data=df, color='blue', jitter=True, alpha=0.8, zorder=1)
ax.set_ylabel('CII')

plt.show()
