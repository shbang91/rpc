import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import warnings
from plot_utils import *
matplotlib.use('TkAgg')

script_directory = os.path.dirname(os.path.abspath(__file__))

lbound_time = 2
ubound_time = 10

"""
def sliceTime(time, lbound, ubound):
    if lbound not in time or ubound not in time:
        print("Error: lbound_time or ubound_time not found in timeCOM vector.")
        print("min time: ", time[0])
        print("max time: " , time[-1])
        if lbound not in time:
            lbound = time[0]
        if ubound not in time:
            ubound = time[-1]
    lbound_idx = np.searchsorted(time, lbound, side='left')
    ubound_idx = np.searchsorted(time, ubound, side='right')

    return slice(lbound_idx, ubound_idx)
"""




def plotTask(file, type, subtitle, lbound, ubound):
    pos, pos_d, vel, vel_d, weight, time, st, phase = read_task(file, type)
    inter = sliceTime(time, lbound, ubound)
    st+=2
    plot_task(time[inter], pos_d[inter], pos[inter], vel_d[inter], vel[inter], st[inter], subtitle)





plotTask('task_com_z.txt', "com_z", "com height task", lbound_time, ubound_time)



plotTask('task_rf_ori.txt', "ori", "rfoot ori task", lbound_time, ubound_time)
plotTask('task_lf_ori.txt', "ori", "lfoot ori task", lbound_time, ubound_time)
plotTask('task_rf_pos.txt', "pos", "rfoot pos task", lbound_time, ubound_time)
plotTask('task_lf_pos.txt', "pos", "lfoot pos task", lbound_time, ubound_time)
plotTask('task_torso_com_ori.txt', "ori", "torso com ori", lbound_time, ubound_time)


"""
u_pos, u_pos_d, u_vel, u_vel_des, u_weight, u_time, u_st, u_phase = read_upper_body('task_upper_body.txt')

for i in range(13):
    plt.figure()
    plt.plot(u_time, u_pos[:,i])
    plt.plot(u_time, u_pos_d[:, i])
"""

plt.show()