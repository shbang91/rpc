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
    # cwd + '/experiment_data/draco_cii_prox.pkl',
    # cwd + '/experiment_data/draco_cii_col.pkl',
    # cwd + '/experiment_data/atlas_cii.pkl',
    # cwd + '/experiment_data/valkyrie_cii.pkl'
    # cwd + '/experiment_data/cassie_cii.pkl',
    # cwd + '/experiment_data/draco_cii_proximal_walking_traj.pkl',
    # cwd + '/experiment_data/draco_cii_collocated_walking_traj.pkl',
    # cwd + '/experiment_data/valkyrie_cii_walking_traj.pkl',
    # cwd + '/experiment_data/hubo_cii_walking_traj.pkl',
    # cwd + '/experiment_data/atlas_cii_walking_traj.pkl'
    cwd + '/experiment_data/draco_cii_proximal_walking_pink.pkl',
    cwd + '/experiment_data/draco_cii_collocated_walking_pink.pkl',
    cwd + '/experiment_data/ergocub_cii_walking_pink.pkl',
    cwd + '/experiment_data/icub_cii_walking_pink.pkl',
    cwd + '/experiment_data/hubo_cii_walking_pink.pkl',
    cwd + '/experiment_data/valkyrie_cii_walking_traj.pkl',
    cwd + '/experiment_data/atlas_cii_walking_pink.pkl'
]

draco_cii_proximal = []
draco_cii_proximal_pink = []
draco_cii_collocated = []
draco_cii_collocated_pink = []
atlas_cii = []
valkyrie_cii = []
cassie_cii = []
draco_cii_one_step = []
draco_cii_one_step_collocated = []
valkyrie_cii_one_step = []
hubo_cii_one_step = []
atlas_cii_one_step = []
atlas_cii_pink = []
ergocub_cii_pink = []
icub_cii_pink = []
hubo_cii_pink = []

for cii_file in cii_file_list:
    with open(cii_file, 'rb') as file:
        while True:
            try:
                data = pickle.load(file)
                if cii_file == cwd + '/experiment_data/draco_cii_prox.pkl':
                    draco_cii_proximal.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/draco_cii_col.pkl':
                    draco_cii_collocated.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/draco_cii_collocated_walking_pink.pkl':
                    draco_cii_collocated_pink.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/draco_cii_proximal_walking_pink.pkl':
                    draco_cii_proximal_pink.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/atlas_cii_walking_pink.pkl':
                    atlas_cii_pink.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/ergocub_cii_walking_pink.pkl':
                    ergocub_cii_pink.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/icub_cii_walking_pink.pkl':
                    icub_cii_pink.append(data['cii'])
                elif cii_file == cwd + '/experiment_data/hubo_cii_walking_pink.pkl':
                    hubo_cii_pink.append(data['cii'])
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
                    hubo_cii_one_step.append(-1 * data['cii'])
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
    # 'atlas': atlas_cii_one_step,
    # 'valkyrie': valkyrie_cii_one_step,
    # 'hubo plus': hubo_cii_one_step,
    # 'draco(proximal)': draco_cii_one_step,
    # 'draco(collocated)': draco_cii_one_step_collocated
    # 'atlas': atlas_cii_pink,
    # 'valkyrie': valkyrie_cii_one_step,
    'Hubo2 Plus': hubo_cii_pink,
    'draco(proximal)': draco_cii_proximal_pink,
    'draco(collocated)': draco_cii_collocated_pink,
    'icub': icub_cii_pink,
    'ergocub': ergocub_cii_pink
}

# print(len(draco_cii_proximal_pink))
# print(len(draco_cii_collocated_pink))
# print(len(atlas_cii_pink))
# print(len(valkyrie_cii))
# print(len(atlas_cii_one_step))

# print("========Atlas========")
# max_val = max(atlas_cii_one_step)
# min_val = min(atlas_cii_one_step)
# print("max", max(atlas_cii_one_step))
# print("min", min(atlas_cii_one_step))
# print(np.log(max_val - min_val))
# print("========Hubo========")
# max_val = max(hubo_cii_one_step)
# min_val = min(hubo_cii_one_step)
# print("max", max(hubo_cii_one_step))
# print("min", min(hubo_cii_one_step))
# print(np.log(max_val - min_val))
# print("========Valkyrie========")
# max_val = max(valkyrie_cii_one_step)
# min_val = min(valkyrie_cii_one_step)
# print("max", max(valkyrie_cii_one_step))
# print("min", min(valkyrie_cii_one_step))
# print(np.log(max_val - min_val))
# print("========Draco collocated========")
# max_val = max(draco_cii_one_step_collocated)
# min_val = min(draco_cii_one_step_collocated)
# print("max", max(draco_cii_one_step_collocated))
# print("min", min(draco_cii_one_step_collocated))
# print(np.log(max_val - min_val))
# print("========Draco proximal========")
# max_val = max(draco_cii_one_step)
# min_val = min(draco_cii_one_step)
# print("max", max(draco_cii_one_step))
# print("min", min(draco_cii_one_step))
# print(np.log(max_val - min_val))

# print("========Draco proximal========")
# max_val = max(draco_cii_proximal_pink)
# min_val = min(draco_cii_proximal_pink)
# print("max", max(draco_cii_proximal_pink))
# print("min", min(draco_cii_proximal_pink))
# print(np.log(max_val - min_val))

# print("========atlas========")
# max_val = max(atlas_cii_pink)
# min_val = min(atlas_cii_pink)
# print("max", max(atlas_cii_pink))
# print("min", min(atlas_cii_pink))
# print(np.log(max_val - min_val))

# print("========ergocub========")
# max_val = max(ergocub_cii_pink)
# min_val = min(ergocub_cii_pink)
# print("max", max(ergocub_cii_pink))
# print("min", min(ergocub_cii_pink))
# print(np.log(max_val - min_val))

df = pd.DataFrame(pd_data)

fig, ax = plt.subplots()
ax.grid(True)
# ax = sns.violinplot(data=df, scale='area', width=3.0)
ax = sns.violinplot(data=df, scale='width')
# ax = sns.stripplot(data=df, color='blue', jitter=True, alpha=0.8, zorder=1)
# ax = sns.boxplot(data=df, flierprops={"marker": "x"})
# ax = sns.catplot(data=df, kind="violin", color=".9", inner=None)
# ax = sns.swarmplot(data=df, size=1)
ax.set_ylabel('CII', fontsize=12)

# plt.savefig("group_cii.pdf", format='pdf')

plt.show()
