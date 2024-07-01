import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy
from scipy.linalg import expm
from plot_utils import *
matplotlib.use('TkAgg')
import math
from read_data import *


############################
## Swing Foot 3d tracking ##
############################
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(swingXCommand, swingYCommand, swingZCommand, marker='x', label = 'Command')
ax.plot(trRobotSwing[:,0], trRobotSwing[:,1], trRobotSwing[:,2], marker = 'x', color = 'red', label = 'pos')
ax.legend()

############################
##  Tracking of desired   ##
############################
for i in range(len(time)):
    time[i] = time[i]+1

# MPC evolution vs real L
l = np.sqrt(9.81/MpczH)

Lx_plus = 0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off
Lx_minus = -0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off


print(Lx_plus)
plt.figure()
plt.plot(alip_time, mpc_coor_L_y)
plt.plot(MpctimeCOM, MpcLyCOM)
plt.title('robot Ly  ')
for t in landingTime:

    plt.axvline(x=t-31, color='black', linestyle='-', linewidth=0.5)


plt.figure()
plt.plot(alip_time, mpc_coor_L_x)
plt.plot(MpctimeCOM, MpcLxCOM)
plt.title('robot Lx ')
for t in landingTime:
    plt.axvline(x=t-31, color='black', linestyle='-', linewidth=0.5)

plt.figure()
plt.plot(alip_time, mpc_coor_L_z)
plt.plot(MpctimeCOM, MpcLzCOM)
plt.title('robot Lz ')
for t in landingTime:
    plt.axvline(x=t-31, color='black', linestyle='-', linewidth=0.5)

#########################
#Tracking desired Lx, Ly
#########################
_, _, _, _, _, _, st, _ = read_task('task_com_z.txt', "com_z")
st = st[inter]
st = st[:-2]
    
#Ly des
Ly_m = (mpc_coor_L_y[inter])/(MpcMassCom[inter]*MpczH[inter])
Lydes_m = MpcLy_des[inter]/(MpcMassCom[inter]*MpczH[inter])
plt.figure(figsize=(10, 6))
plt.plot(time[inter], Ly_m, label=r'$L_{y}$')
plt.plot(time[inter], Lydes_m, label=r'$L_{y}^{des}$')
plt.xlabel('Time', fontsize = 20)
plt.ylabel(r'$\frac{L_{y}}{mz_{H}}$', fontsize = 20, rotation='horizontal', labelpad=20)
plt.legend(fontsize=16)
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i]+1, landingTimes_in_range[i+1]+1, color=colors[i % len(colors)], alpha=0.3)
plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 
plt.ylim(-1, 1)


#Lx_des
Lx_m = (mpc_coor_L_x[inter])/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m1 = Lx_plus[inter]/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m2 = Lx_minus[inter]/(MpcMassCom[inter]*MpczH[inter])
plt.figure(figsize=(10, 6))
plt.plot(time[inter], Lx_m, label = r'$L_{x}$')
plt.plot(time[inter], Lxdes_m1, label = r'$L_{x}^{des,p}$')
plt.plot(time[inter], Lxdes_m2, label = r'$L_{x}^{des,m}$')
plt.xlabel('Time (s)', fontsize = 16)
plt.ylabel(r'$\frac{L_{x}}{mz_{H}}$', rotation='horizontal', fontsize = 20, labelpad=20)
plt.legend(fontsize=16)
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i]+1, landingTimes_in_range[i+1]+1, color=colors[i % len(colors)], alpha=0.3)
plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 
#plt.ylim(-1, 1)

###########################
# Tracking desired yaw
###########################
plt.figure(figsize=(10, 6))
plt.plot(time[inter], torso_yaw[inter])
plt.xlabel('Time (s)', fontsize = 16)
plt.ylabel('torso yaw (rad)', fontsize = 16)
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i]+1, landingTimes_in_range[i+1]+1, color=colors[i % len(colors)], alpha=0.3)
plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 
plt.ylim(-math.pi, math.pi)



















plt.show()





