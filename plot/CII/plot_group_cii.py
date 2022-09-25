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
    cwd + '/experiment_data/draco_cii_prox.pkl',
    cwd + '/experiment_data/draco_cii_col.pkl',
    cwd + '/experiment_data/atlas_cii.pkl',
    cwd + '/experiment_data/valkyrie_cii.pkl',
    cwd + '/experiment_data/cassie_cii.pkl',
    cwd + '/experiment_data/draco_cii_proximal_walking_traj.pkl',
    cwd + '/experiment_data/draco_cii_collocated_walking_traj.pkl',
    cwd + '/experiment_data/valkyrie_cii_walking_traj.pkl',
    cwd + '/experiment_data/hubo_cii_walking_traj.pkl',
    cwd + '/experiment_data/atlas_cii_walking_traj.pkl'

]

draco_cii_proximal = []
draco_cii_collocated = []
atlas_cii = []
valkyrie_cii = []
cassie_cii = []
draco_cii_one_step = []
draco_cii_one_step_collocated = []
valkyrie_cii_one_step = []
hubo_cii_one_step = []
atlas_cii_one_step = []

for cii_file in cii_file_list:
    with open(cii_file, 'rb') as file:
        while True:
            try:
                data = pickle.load(file)
                if cii_file == cwd + '/experiment_data/draco_cii_prox.pkl':
                    draco_cii_proximal.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/draco_cii_col.pkl':
                    draco_cii_collocated.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/atlas_cii.pkl':
                    atlas_cii.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/valkyrie_cii.pkl':
                    valkyrie_cii.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/cassie_cii.pkl':
                    cassie_cii.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/draco_cii_proximal_walking_traj.pkl':
                    draco_cii_one_step.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/draco_cii_collocated_walking_traj.pkl':
                    draco_cii_one_step_collocated.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/valkyrie_cii_walking_traj.pkl':
                    valkyrie_cii_one_step.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/hubo_cii_walking_traj.pkl':
                    hubo_cii_one_step.append(-1*data['cii'])
                elif cii_file == cwd + '/experiment_data/atlas_cii_walking_traj.pkl':
                    atlas_cii_one_step.append(-1 * data['cii'])
            except EOFError:
                break

pd_data = {
    # 'draco (proximal actuation)': draco_cii_proximal,
    # 'draco (collocated actuation)': draco_cii_collocated,
    # 'atlas': atlas_cii,
    # 'valkyrie': valkyrie_cii,
    # 'cassie': cassie_cii
    'atlas': atlas_cii_one_step,
    'valkyrie': valkyrie_cii_one_step,
    'hubo plus': hubo_cii_one_step,
    'draco(proximal)': draco_cii_one_step,
    'draco(collocated)': draco_cii_one_step_collocated
}

print(len(draco_cii_one_step))
print(len(draco_cii_one_step_collocated))
print(len(valkyrie_cii_one_step))
print(len(hubo_cii_one_step))
print(len(atlas_cii_one_step))

df = pd.DataFrame(pd_data)

fig, ax = plt.subplots()
ax.grid(True)
ax = sns.violinplot(data=df, color="lightgray", linewidth=3)
ax = sns.stripplot(data=df, color='blue', jitter=True, alpha=0.8, zorder=1)
ax.set_ylabel('CII', fontsize=12)

plt.show()
