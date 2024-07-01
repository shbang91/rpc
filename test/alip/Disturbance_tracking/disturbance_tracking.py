import sys
sys.path.append('/home/carlos/Desktop/Austin/SeungHyeonProject/rpc/test/alip')
from plot_utils import *
from disturbance_test import read_disturbance_file
import os


lbound_time = 2#5
ubound_time = 100#8

script_directory = os.path.dirname(os.path.abspath(__file__))

print(script_directory)

#trSw = merge_swing_traj(trSw1, trSw2)
"""
MpcComState = readRobotSwTr('One_Step/RL/MpcCOMstate.txt')

COMmpcoor = readRobotSwTr('One_Step/RL/RobotCOMmpcOri.txt')

Force = read_disturbance_file('One_Step/RL/disturbance.txt')

with open('One_Step/RL/LandTime.txt', 'r') as file:
    landingTime = [float(line.strip()) for line in file]
"""
MpcComState = readRobotSwTr('One_Step/MPC/MpcCOMstate.txt')
COMmpcoor = readRobotSwTr('One_Step/MPC/RobotCOMmpcOri.txt')
Force = read_disturbance_file('One_Step/MPC/disturbance.txt')
with open('One_Step/RL/LandTime.txt', 'r') as file:
    landingTime = [float(line.strip()) for line in file]


mpc_coor_x = COMmpcoor[:, 0]
mpc_coor_y = COMmpcoor[:, 1]
mpc_coor_z = COMmpcoor[:, 2]
mpc_coor_vx = COMmpcoor[:, 3]
mpc_coor_vy = COMmpcoor[:, 4]
mpc_coor_vz = COMmpcoor[:, 5]
mpc_coor_Lst_x = COMmpcoor[:, 6]
mpc_coor_Lst_y = COMmpcoor[:, 7]
mpc_coor_Lst_z = COMmpcoor[:, 8]
mpc_coor_Lc_x = COMmpcoor[:, 9]
mpc_coor_Lc_y = COMmpcoor[:, 10]
mpc_coor_Lc_z = COMmpcoor[:, 11]
mpc_coor_L_x = COMmpcoor[:,12]
mpc_coor_L_y = COMmpcoor[:,13]
mpc_coor_L_z = COMmpcoor[:,14]
alip_time = COMmpcoor[:,15]

timeCOM = alip_time
time = alip_time

MpcxCOM = MpcComState[:,0]
MpcyCOM = MpcComState[:,1]
MpczCOM = MpcComState[:,2]
MpcLxCOM= MpcComState[:,3]
MpcLyCOM = MpcComState[:,4]
MpcLzCOM = MpcComState[:,5]
MpctimeCOM = MpcComState[:,6]
MpcLx_off = MpcComState[:,7]
MpcLy_des = MpcComState[:,8]
MpcMu = MpcComState[:,9]
MpcLegWidth = MpcComState[:,10]
MpczH = MpcComState[:,11]
MpcMassCom = MpcComState[:,12]
MpcTs = MpcComState[:,13]




inter = sliceTime(time, lbound_time, ubound_time)
landingTimes_in_range = [t for t in landingTime if lbound_time <= t <= ubound_time]

light_grey = [0.85, 0.85, 0.85]
facecolors = [
    light_grey, 'grey', 'brown', 'red', 'orange', 'yellow', 'green', 'blue', 'purple',
    'crimson', 'white'
] 
colors =[facecolors[0], facecolors[-1]]

l = np.sqrt(9.81/MpczH)
Lx_plus = 0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off
Lx_minus = -0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off


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
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 

#Lx_des
Lx_m = (mpc_coor_L_x[inter])/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m1 = Lx_plus[inter]/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m2 = Lx_minus[inter]/(MpcMassCom[inter]*MpczH[inter])
plt.figure(figsize=(10, 6))
plt.plot(time[inter], Lx_m, label = r'$L_{x}$')
plt.plot(time[inter], Lxdes_m1, label = r'$L_{x}^{des,p}$')
plt.plot(time[inter], Lxdes_m2, label = r'$L_{x}^{des,m}$')
plt.xlabel('Time (s)', fontsize = 16)
plt.ylabel(r'$\frac{L_{x}}{mz_{H}} (ms^{-1})$', fontsize = 20, rotation='horizontal', labelpad=20)
plt.legend(fontsize=16)
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 





fig, ax1 = plt.subplots(figsize=(10, 6))

# Plot vy with the first y-axis
color = 'tab:blue'
ax1.set_xlabel('Time (s)', fontsize = 20)
ax1.set_ylabel(r'$\frac{L_{y}}{mz_{H}}$', fontsize = 20, rotation = 'horizontal',color='black')
ax1.plot(time[inter], Ly_m, color=color, label=r'$L_y$')
ax1.tick_params(axis='y', labelcolor='black')

# Plot Lx_m with the first y-axis
color = 'tab:orange'
ax1.plot(time[inter], Lydes_m, color=color, label=r'$L_y^{des}$')
ax1.tick_params(axis='y', labelcolor='black')

# Create a single y-axis for Fx and Fy
ax2 = ax1.twinx()  
ax2.set_ylabel(r'Force (N)', color='black', fontsize = 20) 

print(inter)
start_idx = inter.start+len(Force)-len(time)
end_idx = inter.stop+len(Force)-len(time)
inter_force = slice(start_idx, end_idx)
print(len(time))
print(len(Force))
print(inter_force)
ax2.plot(time[inter], Force[inter_force, 0], color = 'Red', label=r'$F_x$')
ax2.plot(time[inter], Force[inter_force, 1], color = 'Purple',label=r'$F_y$')
ax2.tick_params(axis='y', labelcolor='black')

# Set legend
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2,fontsize=14)

for i in range(len(landingTimes_in_range) - 1):
    ax1.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)

plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 



#Lx

fig, ax1 = plt.subplots(figsize=(10, 6))
Lx_m = (mpc_coor_L_x[inter])/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m1 = Lx_plus[inter]/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m2 = Lx_minus[inter]/(MpcMassCom[inter]*MpczH[inter])

color = 'tab:blue'
ax1.set_xlabel('Time (s)', fontsize = 20)
ax1.set_ylabel(r'$\frac{L_{x}}{mz_{H}}$', fontsize = 20, rotation = 'horizontal',color='black')
ax1.plot(time[inter], Lx_m, color=color, label=r'$L_x$')
ax1.plot(time[inter], Lxdes_m1, label = r'$L_{x}^{des,p}$', color = 'tab:orange')
ax1.plot(time[inter], Lxdes_m2, label = r'$L_{x}^{des,m}$', color = 'tab:green')
ax1.tick_params(axis='y', labelcolor='black')


# Create a single y-axis for Fx and Fy
ax2 = ax1.twinx()  
ax2.set_ylabel(r'Force (N)', color='black', fontsize = 20) 

print(inter)
start_idx = inter.start+len(Force)-len(time)
end_idx = inter.stop+len(Force)-len(time)
inter_force = slice(start_idx, end_idx)
print(len(time))
print(len(Force))
print(inter_force)
ax2.plot(time[inter], Force[inter_force, 0], color = 'tab:red', label=r'$F_x$')
ax2.plot(time[inter], Force[inter_force, 1], color = 'tab:purple',label=r'$F_y$')
ax2.tick_params(axis='y', labelcolor='black')

# Set legend
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, fontsize=14)

for i in range(len(landingTimes_in_range) - 1):
    ax1.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)

plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 













plt.show()
