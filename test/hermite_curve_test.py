import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

from util.python_utils import interpolation
import ipdb

import numpy as np

import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

if __name__ == "__main__":

    ## hermiteCurve Test
    # p1 = 0.2
    # v1 = 0.
    # p2 = 0.7
    # v2 = 0.
    # duration = 2.

    # curve = interpolation.HermiteCurve(p1, v1, p2, v2, duration)

    # time_list, pos_list, vel_list, acc_list = [], [], [], []
    # N = 100
    # for i in range(N):
    # t = duration * i / N
    # time_list.append(t)
    # pos_list.append(curve.evaluate(t))
    # vel_list.append(curve.evaluate_first_derivative(t))
    # acc_list.append(curve.evaluate_second_derivative(t))

    # figure, axes = plt.subplots(3, 1)
    # axes[0].plot(time_list, pos_list)
    # axes[0].grid(True)
    # axes[1].plot(time_list, vel_list)
    # axes[1].grid(True)
    # axes[2].plot(time_list, acc_list)
    # axes[2].grid(True)

    # plt.show()

    ## hermiteCurveVec Test
    # p1 = np.array([1., 1., 1.])
    # v1 = np.array([0., 0., 0.])
    # p2 = np.array([3., 3., 3.])
    # v2 = np.array([0., 0., 0.])
    # duration = 2.

    # curveVec = interpolation.HermiteCurveVec(p1, v1, p2, v2, duration)

    # time_list = []
    # pos_list, vel_list, acc_list = [], [], []
    # N = 100
    # for i in range(N):
    # t = duration * i / N
    # time_list.append(t)
    # pos_list.append(curveVec.evaluate(t))
    # vel_list.append(curveVec.evaluate_first_derivative(t))
    # acc_list.append(curveVec.evaluate_second_derivative(t))

    # figure, axes = plt.subplots(3, 3)
    # for i in range(3):
    # axes[0][i].plot(time_list, [pos[i] for pos in pos_list])
    # axes[0][i].grid(True)
    # axes[1][i].plot(time_list, [vel[i] for vel in vel_list])
    # axes[1][i].grid(True)
    # axes[2][i].plot(time_list, [acc[i] for acc in acc_list])
    # axes[2][i].grid(True)

    # plt.show()

    ## hermiteCurveQuat
    p1 = np.array([0., 0., 0., 1.])
    v1 = np.array([0., 0., 0.])
    p2 = np.array([1., 0., 0., 0.])
    v2 = np.array([0., 0., 0.])
    duration = 2.

    curveQuat = interpolation.HermiteCurveQuat(p1, v1, p2, v2, duration)

    time_list = []
    pos_list, vel_list, acc_list = [], [], []
    N = 100
    for i in range(N):
        t = duration * i / N
        time_list.append(t)
        pos_list.append(curveQuat.evaluate(t))
        vel_list.append(curveQuat.evaluate_ang_vel(t))
        acc_list.append(curveQuat.evaluate_ang_acc(t))

    figure_1, axes_1 = plt.subplots(4, 1)
    for i in range(4):
        axes_1[i].plot(time_list, [pos[i] for pos in pos_list])
        axes_1[i].grid(True)

    figure_2, axes_2 = plt.subplots(3, 1)
    for i in range(3):
        axes_2[i].plot(time_list, [vel[i] for vel in vel_list])
        axes_2[i].grid(True)

    figure_3, axes_3 = plt.subplots(3, 1)
    for i in range(3):
        axes_3[i].plot(time_list, [acc[i] for acc in acc_list])
        axes_3[i].grid(True)

    plt.show()

    # figure, axes = plt.subplots(3, 3)
    # for i in range(3):
    # axes[0][i].plot(time_list, [pos[i] for pos in pos_list])
    # axes[0][i].grid(True)
    # axes[1][i].plot(time_list, [vel[i] for vel in vel_list])
    # axes[1][i].grid(True)
    # axes[2][i].plot(time_list, [acc[i] for acc in acc_list])
    # axes[2][i].grid(True)

    plt.show()
