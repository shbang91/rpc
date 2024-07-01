import random
import warnings

import numpy as np
import matplotlib
#matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


xyz_label = ['x', 'y', 'z']
quat_label = ['x', ' y', ' z', 'w']
markers = ['*', '+', 'h', 'x', 'o', 'v', 'd']
facecolors = [
    'grey', 'brown', 'red', 'orange', 'yellow', 'green', 'blue', 'purple',
    'crimson'
] * 10


def plot_task(time, pos_des, pos, vel_des, vel, phase, suptitle, leg_switch_time = None):
    if pos_des.shape[1] == 3:

        fig, axes = plt.subplots(3, 2)
        for i in range(3):
            axes[i, 0].tick_params(axis = 'both', which='major', labelsize='14')
            axes[i, 1].tick_params(axis = 'both', which='major', labelsize='14')

            axes[i, 0].plot(time,
                            pos_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 0].plot(time, pos[:, i], color='b', linewidth=2)
            axes[i, 0].grid(True)
            axes[i, 0].set_ylabel(xyz_label[i], fontsize=16)
            plot_phase(axes[i, 0], time, phase)
            axes[i, 1].plot(time,
                            vel_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 1].plot(time, vel[:, i], color='b', linewidth=2)
            axes[i, 1].grid(True)
            axes[i, 1].set_ylabel(r'$\dot{' + xyz_label[i] + '}$', fontsize=14)
            plot_phase(axes[i, 1], time, phase)
            if leg_switch_time is not None:
                for t in leg_switch_time:
                    axes[i, 1].axvline(x=t, color='k', linestyle='--')  # Add vertical line at leg_switch_time 
                    axes[i, 0].axvline(x=t, color='k', linestyle='--')  # Add vertical line at leg_switch_time    
   
        axes[2, 0].set_xlabel('time', fontsize=16)
        axes[2, 1].set_xlabel('time', fontsize=16)
        fig.suptitle(suptitle)

    elif pos_des.shape[1] == 4:
        fig, axes = plt.subplots(4, 2)
        plt.tick_params(axis='both', which='major', labelsize=12)
        for i in range(4):
            axes[i, 0].tick_params(axis = 'both', which='major', labelsize='14')
            axes[i, 0].plot(time,
                            pos_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 0].plot(time, pos[:, i], color='b', linewidth=2)
            axes[i, 0].grid(True)
            axes[i, 0].set_ylabel(quat_label[i],fontsize=16)
            axes[i,0].set_xbound(1, -1)
            plot_phase(axes[i, 0], time, phase)
        for i in range(3):
            axes[i, 1].tick_params(axis = 'both', which='major', labelsize='14')
            axes[i, 1].plot(time,
                            vel_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 1].plot(time, vel[:, i], color='b', linewidth=2)
            plot_phase(axes[i, 1], time, phase)
            axes[i, 1].grid(True)
            axes[i, 1].set_ylabel(r'$\dot{' + xyz_label[i] + '}$',fontsize=16)
        axes[3, 0].set_xlabel('time',fontsize=16)
        axes[3, 1].set_xlabel('time', fontsize=16)
        fig.suptitle(suptitle)

    elif pos_des.shape[1] == 1:
        print("hee")
        dim = pos_des.shape[1]
        fig, axes = plt.subplots(dim, 2, figsize=(20,5))
        axes[0].tick_params(axis = 'both', which='major', labelsize='14')
        axes[1].tick_params(axis = 'both', which='major', labelsize='14')
        axes[0].plot(time, pos_des, color='r', linestyle='dashed', linewidth=4)
        axes[0].plot(time, pos, color='b', linewidth=2)
        plot_phase(axes[0], time, phase)
        axes[0].grid(True)
        axes[1].plot(time, vel_des, color='r', linestyle='dashed', linewidth=4)
        axes[1].plot(time, vel, color='b', linewidth=2)
        plot_phase(axes[1], time, phase) 
        axes[1].grid(True)
        axes[1].set_ylabel(r'$\mathrm{\dot{z}}$',fontsize=16)
        axes[0].set_ylabel(r'$\mathrm{z}$',fontsize=16)

        axes[0].set_xlabel('time')
        axes[1].set_xlabel('time')
        fig.suptitle(suptitle)

    else:
        print("hee2")
        dim = pos_des.shape[1]
        fig, axes = plt.subplots(dim, 2)
        plt.tick_params(axis='both', which='major', labelsize=12)
        for i in range(dim):
            axes[i, 0].plot(time,
                            pos_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 0].plot(time, pos[:, i], color='b', linewidth=2)
            plot_phase(axes[i, 0], time, phase)
            axes[i, 0].grid(True)
            axes[i, 1].plot(time,
                            vel_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 1].plot(time, vel[:, i], color='b', linewidth=2)
            plot_phase(axes[i, 1], time, phase)
            axes[i, 1].grid(True)
 
        axes[dim - 1, 0].set_xlabel('time')
        axes[dim - 1, 1].set_xlabel('time')
        fig.suptitle(suptitle)


def plot_weights(time, weights_dict, phase):
    fig, ax = plt.subplots()
    plt.tick_params(axis='both', which='major', labelsize=12)
    for i, (k, v) in enumerate(weights_dict.items()):
        ax.plot(time,
                v,
                label=k,
                marker=markers[i],
                markersize=10,
                markevery=random.randint(50, 150))
    plot_phase(ax, time, phase)
    ax.grid(True)
    ax.set_xlabel('time')
    ax.legend()
    fig.suptitle('task weights')



def plot_phase(ax, t, data_phse):
    phseChange = []
    print(data_phse)
    for i in range(0, len(t) - 1):
        if data_phse[i] != data_phse[i + 1]:
            phseChange.append(i)
        else:
            pass
    print(phseChange)
    shading = 0.2
    prev_j = 0
    ll, ul = (ax.get_ylim())
    i = 0
    for j in (phseChange):
        print(data_phse[j])
        i+=1
        if (i == 30): exit()
        ax.fill_between(t[prev_j:j + 1],
                        ll,
                        ul,
                        facecolor=facecolors[data_phse[j]],
                        alpha=shading)
        prev_j = j
    ax.fill_between(t[prev_j:],
                    ll,
                    ul,
                    facecolor=facecolors[data_phse[prev_j]],
                    alpha=shading)


def read_task(file_path, task_type): #task type = "ori", "pos", "com_z"
    if task_type == "com_z":
        inc = 1
        i = 1
    elif task_type == "ori":
        inc = 3
        i = 4
    elif task_type == "pos":
        inc = 3
        i = 3

    trajectories = np.zeros(2*i + 3*inc +3).reshape(1, -1)
    with open(file_path, 'r') as file:
        for line in file:
            values = [float(val) for val in line.split()]
            values_ = np.array(values).reshape(1, -1)
            trajectories = np.concatenate((trajectories, values_), axis = 0)        



    pos = trajectories[:, 0:i]
    des_pos = trajectories[:, i:i+i]
    i+=i
    vel = trajectories[:, i:i+inc]
    i+=inc
    des_vel = trajectories[:, i:i+inc]
    i+=inc
    weight = trajectories[:, i:i+inc]
    i += inc
    time = trajectories[:,i]
    st_leg = trajectories[:,i+1].astype(int)
    phase = trajectories[:, i+2].astype(int)
    return pos, des_pos, vel, des_vel, weight, time, st_leg, phase 


def read_upper_body(file_path):
    trajectories = np.zeros(13*5 + 3).reshape(1, -1)

    with open(file_path, 'r') as file:
        for line in file:
            values = [float(val) for val in line.split()]
            values_ = np.array(values).reshape(1, -1)
            trajectories = np.concatenate((trajectories, values_), axis = 0)        

    pos = trajectories[:, 0:13]
    des_pos = trajectories[:, 13:26]
    vel = trajectories[:, 26:39]
    des_vel = trajectories[:, 39:52]
    weights = trajectories[:, 52:65]
    time = trajectories[:, 65]
    st_leg = trajectories[:, 66]
    phase = trajectories[:, 67]


    return  pos, des_pos, vel, des_vel, weights, time, st_leg, phase



def all_trajectories(file_path):
    trajectories = []
    current_trajectory = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('end'):
                if current_trajectory:
                    trajectories.append(np.array(current_trajectory))
                else:
                    trajectories.append(np.array([0,0,0,0]))
            elif line.startswith('start'):
                current_trajectory = []
            else:
                values = [float(val) for val in line.split()]
                current_trajectory.append(values)
    
    return trajectories



def readRobotSwTr(file_path):
    trajectories = []
    with open(file_path, 'r') as file:
        for line in file:
            values = [float(val) for val in line.split()]
            trajectories.append(values)  
    trajectories = np.array(trajectories)  
    return trajectories


def sliceTime(time, lbound, ubound):
    if lbound > time[-1]:
        warnings.warn("Warning: lbound ")
        print(time[0], " ", time[-1])
    if ubound > time[-1]:
        warnings.warn("Warning: ubound ")
        print(time[0], " ", time[-1])

    
    # Find the index of the closest time to lbound
    lbound_idx = np.argmin(np.abs(np.array(time) - lbound))
    
    # Find the index of the closest time to ubound
    ubound_idx = np.argmin(np.abs(np.array(time) - ubound))
    
    return slice(lbound_idx, ubound_idx+1)  # Adding 1 to include ubound in the slice

