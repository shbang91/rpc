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
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
from stable_baselines3.common.monitor import Monitor

from util.python_utils.util import read_config

from simulator.pybullet.rl.rl_mpc_freq.envs.freq_env_tilted_ground_5_Ly_range_max_iter_150 import DracoEnvMpcFreq_tilted_ground_Ly_range_max_iter_150

if __name__ == "__main__":
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    load_path = os.path.join(
        cwd,
        'rl_model/freq_env/tilted_ground_Ly_range_max_iter_150/PPO/redObsLy_range_tilted_5_downhill'
    )

    CURR_TIMESTEP = 17000000
    model_name = f'_TIME{CURR_TIMESTEP}.zip'
    norm_name = f'TIME{CURR_TIMESTEP}.pkl'
    norm_path = os.path.join(load_path, norm_name)
    load_path = os.path.join(load_path, model_name)

    reduced_obs_size = True
    mpc_freq = 5
    sim_dt = Config.CONTROLLER_DT

    env = DracoEnvMpcFreq_tilted_ground_Ly_range_max_iter_150(
        mpc_freq,
        sim_dt,
        eval=[0, 0, 0],
        reduced_obs_size=reduced_obs_size,
        render=True)
    # video='freq_env_MPC_Ly_range_auto_2.mp4')
    monitor_env = Monitor(env)
    vec_env = DummyVecEnv([lambda: monitor_env])
    norm_env = VecNormalize.load(norm_path, vec_env)
    norm_env.training = False
    norm_env.norm_reward = False

    obs, info = env.reset()
    obs = norm_env.normalize_obs(obs)

    model = PPO.load(load_path, env=norm_env)
    des_counter = 0
    # Ly_des = [0, 5, 10, 15, 20, 25, 0, -5, -10, -15, -20, -25]
    # Ly_des = [0, 5, 10, 15, 20, 10, 0, -5, -10, -15, -20, -25]
    Ly_des = [0, 5, 10, 15, 20]  #TODO: it's working
    # Ly_des = [0, -5, -10, -15, -20] #TODO: it's working
    #Leg Width must be 0.1
    step_counter = 0
    while True:
        step_counter += 1
        #action = torch.ones(AlipParams.N_BATCH,3)
        action, _ = model.predict(obs, deterministic=True)
        #action = 0*action
        config = read_config(cwd + '/config/draco/alip_command.ini')

        ini_st_leg = np.random.choice([1, -1])
        env._set_command_policy_sim(0, Ly_des[des_counter], 0, ini_st_leg)
        print("action: ", env._normalise_action(action))
        obs, reward, done, trunc, info = env.step(action)
        obs = norm_env.normalize_obs(obs)
        print("reward", reward)
        print("reward info", info["reward_components"])
        if done:
            print(done)
            obs, info = env.reset()
            print(obs)
            obs = norm_env.normalize_obs(obs)
            print(obs)
        if step_counter > 32 * 20:
            des_counter += 1
            step_counter = 0
            if des_counter >= len(Ly_des):
                break
            #env._set_command_policy_sim(0, Ly_des[des_counter], 0, ini_st_leg)
            #obs,info = env.reset()
            #obs = norm_env.normalize_obs(obs)
