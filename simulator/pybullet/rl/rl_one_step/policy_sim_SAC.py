import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
import torch

from stable_baselines3 import SAC

import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
import time, math
from collections import OrderedDict
import copy
import signal
import shutil

import cv2

from config.draco.pybullet_simulation import Config
from simulator.pybullet.rl.rl_one_step.one_step_env import DracoEnvOneStepMpc


if __name__ == "__main__":

    from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    


    model_dir = cwd + "/rl_model/one_step/SAC/"

    yaw_max = 20
    Lx = 0.
    Ly = 0.
    randomized_command = False
    reduced_obs_size = True
    mpc_freq = 0
    sim_dt = Config.CONTROLLER_DT
    render = True

    env = DracoEnvOneStepMpc(Lx, Ly, 15, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = render)

    obs, info = env.reset()
    interface = info["interface"]

    n_steps_ = 256 #512
    batch_size_ = 64
    learning_rate_ = 0.0003

    if reduced_obs_size: str1 = 'redOBS'
    else: str1 = 'fullOBS'
    if randomized_command: str2 = 'randCOMMAND'
    else: str2 = 'detCOMMAND'
    buffer_size_ = 1000000
    learning_starts_ = 100
    batch_size_ = 256


    CURR_TIMESTEP = 280000
    save_dir = str1 + str2 + f"mpc_freq{mpc_freq}_SIMdt{sim_dt}_Lx_{Lx}_Ly_{Ly}_Yaw_{yaw_max}_changed params/"  
    save_subdir = f"lstarts{learning_starts_}buffSize{buffer_size_}LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"


    model_path = model_dir + save_dir + save_subdir
    model = SAC.load(model_path, env=env)

    plot = False

    if plot == False:
        while True:
            #action = torch.ones(AlipParams.N_BATCH,3)
            action, _ = model.predict(obs, deterministic=True)
            print(action)
            obs, reward, done, trunc, info = env.step(action)
            print("reward", reward)
            print("reward info", info["reward_components"])
            if done:
                print(done)
                obs,info = env.reset()
    else:
        while not done:
            action, _ = model.predict(obs, deterministic=False)
            print(action)
            obs, reward, done, trunc, info = env.step(action)