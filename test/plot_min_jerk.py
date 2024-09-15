import sys
import os
import numpy as np
import matplotlib.pyplot as plt

cwd = os.getcwd()
sys.path.append(cwd)

if __name__ == "__main__":
    # load file
    time = np.loadtxt(cwd + "/experiment_data/time.txt", unpack=True)
    pos = np.loadtxt(cwd + "/experiment_data/pos.txt", unpack=True)
    vel = np.loadtxt(cwd + "/experiment_data/vel.txt", unpack=True)
    acc = np.loadtxt(cwd + "/experiment_data/acc.txt", unpack=True)

    # plot
    fig, ax = plt.subplots(3)
    fig.suptitle("Minimum Jerk Interpolation")
    ax[0].plot(time, pos, marker="o", c="g")
    ax[0].set_title("pos")
    ax[0].grid(True)
    ax[1].plot(time, vel, marker="*", c="k")
    ax[1].set_title("vel")
    ax[1].grid(True)
    ax[2].plot(time, acc, marker="+", c="r")
    ax[2].set_title("acc")
    ax[2].grid(True)

    # load file
    pos_cos = np.loadtxt(cwd + "/experiment_data/pos_cos.txt", unpack=True)
    vel_cos = np.loadtxt(cwd + "/experiment_data/vel_cos.txt", unpack=True)
    acc_cos = np.loadtxt(cwd + "/experiment_data/acc_cos.txt", unpack=True)

    fig, ax = plt.subplots(3)
    fig.suptitle("Cosine Interpolation")
    ax[0].plot(time, pos_cos, marker="o", c="g")
    ax[0].set_title("pos")
    ax[0].grid(True)
    ax[1].plot(time, vel_cos, marker="*", c="k")
    ax[1].set_title("vel")
    ax[1].grid(True)
    ax[2].plot(time, acc_cos, marker="+", c="r")
    ax[2].set_title("acc")
    ax[2].grid(True)
    plt.show()
