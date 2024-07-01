import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
import torch

from stable_baselines3 import PPO

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
from simulator.pybullet.rl.env import DracoEnv


if __name__ == "__main__":
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)


    yaw_max = 0
    Lx = 0.
    Ly = 10.
    randomized_command = False
    reduced_obs_size = False
    mpc_freq = 10
    sim_dt = Config.CONTROLLER_DT

    env = DracoEnv(Lx, Ly, yaw_max, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = True)
    obs, info = env.reset()
    interface = info["interface"]
    
    
    n_steps_ = 2048*4 #512
    batch_size_ = 64*4
    learning_rate_ = 0.0003

    if reduced_obs_size: str1 = 'redOBS'
    else: str1 = 'fullOBS'
    if randomized_command: str2 = 'randCOMMAND'
    else: str2 = 'detCOMMAND'

    CURR_TIMESTEP = 1236992

    model_dir = cwd + "/rl_model/PPO/"
    save_dir = str1 + str2 + f"mpc_freq{mpc_freq}_SIMdt{sim_dt}_Lx_{Lx}_Ly_{Ly}_Yaw_{yaw_max}"  + "/"       
    save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"


    model_path = model_dir + save_dir + save_subdir
    model = PPO.load(model_path, env=env)

    plot = False

    if plot == False:
        while True:
            #action = torch.ones(AlipParams.N_BATCH,3)
            action, _ = model.predict(obs, deterministic=True)
            print("action: ", action)
            action *= 0
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