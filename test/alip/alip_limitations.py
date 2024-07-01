import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy
from scipy.linalg import expm
from plot_utils import *

matplotlib.use('TkAgg')

from read_data import *
"""
#########################################################
## Plots ALIP continuous trajectory vs real trajectory ##
#########################################################
def alip_trajectory(x0, dt): #x = [x y Lx Ly]
    mass = 39.15342
    zH = 0.69
    g = 9.81
    A = np.array([[ 0, 0 , 0, 1/(mass*zH)],
                 [0, 0, -1/(mass*zH), 0],
                 [0, -mass*g, 0, 0],
                 [mass*g , 0, 0, 0]])
    Adt = expm(A*dt)
    res = np.matmul(Adt, x0)
    return res
"""
traj_indices = np.where(np.isin(time, landingTimes_in_range))[0]
"""
alip_state_traj = []
alip_time_traj = []
for i in range(np.size(traj_indices)-2):
    x0 = np.array([MpcxCOM[traj_indices[i]+4],
                   MpcyCOM[traj_indices[i]+4],
                   MpcLxCOM[traj_indices[i]+4],
                   MpcLyCOM[traj_indices[i]+4]])
    for j in range(traj_indices[i]+4, traj_indices[i+1]):
        dt = time[j] - time[traj_indices[i]]
        res = alip_trajectory(x0, dt)
        alip_state_traj.append(res)
        alip_time_traj.append(time[j])

def compute_intra_step_traj(start_time, end_time):
    minitraj = []
    minitraj_time = []
    mini_traj_x0 = np.array([mpc_coor_x[start_time],
                            mpc_coor_y[start_time],
                            mpc_coor_L_x[start_time],
                            mpc_coor_L_y[start_time]])

    for j in range(start_time, end_time):
        dt = time[j] - time[start_time]
        res = alip_trajectory(mini_traj_x0, dt)
        minitraj.append(res)
        minitraj_time.append(time[j])

    minitraj = np.stack(minitraj)
    minitraj_time = np.array(minitraj_time)
    return minitraj_time, minitraj


alip_state_traj = np.stack(alip_state_traj)
alip_time_traj = np.array(alip_time_traj)
"""
"""
def plot_alip_traj_vs_real(dim, mpc_coor_, name):
    plt.figure()
    plt.plot(time[traj_indices[0]+5:traj_indices[-2]], mpc_coor_[traj_indices[0]+5:traj_indices[-2]], label = "real " + name)
    plt.plot(alip_time_traj, alip_state_traj[:,dim], label = "alip " + name )
    for i in range(len(landingTimes_in_range) - 1):
        plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
    plt.legend()
"""
"""


#left = 0.145
def plot_alip_traj_vs_real(dim, mpc_coor_, name_robot, name_pred, axis_y, eps1=1e-4, eps2=1e-4):
    plt.figure(figsize=(9, 6))
    #plt.plot(time[traj_indices[0]+5:traj_indices[-2]], mpc_coor_[traj_indices[0]+5:traj_indices[-2]], label="real " + name)
    real_traj_mask = np.abs(np.diff(mpc_coor_[traj_indices[0]:traj_indices[-2]], prepend=mpc_coor_[traj_indices[0]])) < eps1

    # Plot filtered real trajectory
    #plt.plot(time[traj_indices[0]:traj_indices[-2]], np.where(real_traj_mask, mpc_coor_[traj_indices[0]:traj_indices[-2]], np.nan), label=name_robot)
    plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_[traj_indices[0]:traj_indices[-2]], label=name_robot)
    # Calculate mask for points with discontinuities greater than epsilon
    mask = np.abs(np.diff(alip_state_traj[:, dim], prepend=alip_state_traj[0, dim])) < eps2

    # Plot ALIP trajectory with discontinuities filtered
    alip_time_filtered = np.where(mask, alip_time_traj, np.nan)
    alip_state_filtered = np.where(mask, alip_state_traj[:, dim], np.nan)
    #plt.plot(alip_time_traj, alip_state_filtered, label=name_pred)

    plt.plot(alip_time_traj, alip_state_traj[:, dim], label=name_pred)
    for i in range(len(landingTimes_in_range) - 1):
        plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
    plt.xlabel('Time (s)', fontsize = 20)
    plt.ylabel(axis_y, fontsize = 20, labelpad=20)
    plt.legend(loc='upper right', fontsize=20)
    plt.subplots_adjust(left=0.145)

# Plot alip state
plot_alip_traj_vs_real(0, mpc_coor_x, name_robot="$x$", name_pred="$x^{pred}$", axis_y="$x \,(m)$", eps1 = 1e-3, eps2=  1e-3)
plot_alip_traj_vs_real(1, mpc_coor_y, name_robot="$y$", name_pred="$y^{pred}$", axis_y="$y \,(m)$", eps1= 1e-2, eps2=1e-2)
plot_alip_traj_vs_real(2, mpc_coor_L_x, name_robot="$L_{x}$", name_pred="$L_{x}^{pred}$", axis_y="$L_{x} \, (kg \dot m^{2} \dot s^{-1})$", eps1 = 1e-1, eps2 = 1e-1)
plot_alip_traj_vs_real(3, mpc_coor_L_y, name_robot="$L_{y}$", name_pred="$L_{y}^{pred}$", axis_y="$L_{y} \, (kg \dot m^{2} \dot s^{-1})$",eps1=0.075, eps2=0.05)

#neglecting L^x_c, L^y_c
plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_x[traj_indices[0]:traj_indices[-2]], label = '$L^{x}$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_Lc_x[traj_indices[0]:traj_indices[-2]], label = '$L^{x}_{c}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 20)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 20, labelpad=20)
plt.legend(loc='upper right', fontsize=20)
plt.subplots_adjust(left=0.145)


plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_y[traj_indices[0]:traj_indices[-2]], label = '$L^{y}$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_Lc_y[traj_indices[0]:traj_indices[-2]], label = '$L^{y}_{c}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 20)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 20, labelpad=20)
plt.legend(loc='upper right', fontsize=20)
plt.subplots_adjust(left=0.145)


plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_z[traj_indices[0]:traj_indices[-2]], label = '$L^{z}$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_Lc_z[traj_indices[0]:traj_indices[-2]], label = '$L^{z}_{c}$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_z[traj_indices[0]:traj_indices[-2]]-mpc_coor_Lc_z[traj_indices[0]:traj_indices[-2]], label = '$L^{z}-L^{z}_{c}$')

for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 20)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 20, labelpad=20)
plt.legend(loc='upper right', fontsize=20)
plt.subplots_adjust(left=0.145)


plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_z[traj_indices[0]:traj_indices[-2]]-mpc_coor_Lc_z[traj_indices[0]:traj_indices[-2]], label = '$L^{z}-L^{z}_{c}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 20)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 20, labelpad=20)
plt.legend(loc='upper right', fontsize=20)
plt.subplots_adjust(left=0.145)

plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_z[traj_indices[0]:traj_indices[-2]]-0.003545, label = '$z$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], MpczCOM[traj_indices[0]:traj_indices[-2]], label = '$z_{H}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 20)
plt.ylabel('$m$', fontsize = 20, labelpad=20)
plt.legend(loc='upper right', fontsize=20)
plt.subplots_adjust(left=0.145)


"""

######################################
### Plot mpc freq output #############
######################################
one_inter = [i for i in range(traj_indices[10] + 4, traj_indices[11])]

sl = slice(one_inter[0], one_inter[-1], 4)
#End foot command
plt.figure(figsize=(9, 6))
plt.scatter(end_foot_commandx[sl], end_foot_commandy[sl], marker='x')
plt.xlabel('Time (s)', fontsize=20)
plt.ylabel("$L_{y} \, (kg \dot m^{2} \dot s^{-1})$", fontsize=20, labelpad=20)
plt.legend(fontsize=20)
plt.subplots_adjust(left=0.145)

plt.figure(figsize=(9, 6))
plt.scatter(time[sl],
            end_foot_commandx[sl],
            marker='x',
            label='$u_{fp}^{x}(t)$')
plt.xlabel('Time (s)', fontsize=20)
plt.ylabel("$x (m)$", fontsize=20, labelpad=20)
plt.legend(fontsize=20)
plt.subplots_adjust(left=0.145)

plt.figure(figsize=(9, 6))
plt.scatter(time[sl],
            end_foot_commandy[sl],
            marker='x',
            label='$u_{fp}^{y}(t)$')
plt.xlabel('Time (s)', fontsize=20)
plt.ylabel("$y (m)$", fontsize=20, labelpad=20)
plt.legend(fontsize=20)
plt.subplots_adjust(left=0.145)

print(plt.rcParams['axes.prop_cycle'].by_key()['color'])
"""

############################
###### Plot minitraj #######
############################
seq = int((traj_indices[2]-traj_indices[1]-4)/8)
print(traj_indices[1], traj_indices[2])
t0, traj0 = compute_intra_step_traj(traj_indices[1]+4, traj_indices[2])
t1, traj1 = compute_intra_step_traj(traj_indices[1]+seq, traj_indices[2])
t2, traj2 = compute_intra_step_traj(traj_indices[1]+2*seq, traj_indices[2])
t3, traj3 = compute_intra_step_traj(traj_indices[1]+3*seq, traj_indices[2])
t4, traj4 = compute_intra_step_traj(traj_indices[1]+4*seq, traj_indices[2])
t5, traj5 = compute_intra_step_traj(traj_indices[1]+5*seq, traj_indices[2])
t6, traj6 = compute_intra_step_traj(traj_indices[1]+6*seq, traj_indices[2])
t7, traj7 = compute_intra_step_traj(traj_indices[1]+7*seq, traj_indices[2])
t8, traj8 = compute_intra_step_traj(traj_indices[1]+8*seq, traj_indices[2])

plt.figure(figsize=(9, 6))
plt.plot(time[one_inter], mpc_coor_L_y[one_inter], color = '#1f77b4', label='$L_y$')
plt.plot(t0, traj0[:,3], color = '#ff7f0e', label = '$L_y^{pred_0}$')
plt.plot(t1, traj1[:,3], color = '#ff7f0e', label = '$L_y^{pred_i}$',linestyle='dashed')
plt.plot(t2, traj2[:,3], color = '#ff7f0e', linestyle='dashed')
plt.plot(t3, traj3[:,3], color = '#ff7f0e', linestyle='dashed')
plt.plot(t4, traj4[:,3], color = '#ff7f0e', linestyle='dashed')
plt.plot(t5, traj5[:,3], color = '#ff7f0e', linestyle='dashed')
plt.plot(t6, traj6[:,3], color = '#ff7f0e', linestyle='dashed')
plt.plot(t7, traj7[:,3], color = '#ff7f0e', linestyle='dashed')
plt.plot(t8, traj8[:,3], color = '#ff7f0e', linestyle='dashed')
plt.xlabel('Time (s)', fontsize = 20)
plt.ylabel("$L_{y} \, (kg \dot m^{2} \dot s^{-1})$", fontsize = 20, labelpad=20)
plt.legend(fontsize=20)
plt.subplots_adjust(left=0.145)

############################
## Swing Foot 3d tracking ##
############################
"""
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(swingXCommand[one_inter],
        swingYCommand[one_inter],
        swingZCommand[one_inter],
        marker='x',
        label='Reference trajectory')
ax.plot(trRobotSwing[one_inter, 0],
        trRobotSwing[one_inter, 1],
        trRobotSwing[one_inter, 2],
        marker='x',
        color='red',
        label='Position')
ax.legend()

plt.show()
