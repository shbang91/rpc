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

# from simulator.pybullet.rl.rl_mpc_freq.envs.freq_env_Ly_Range_new_reward import DracoEnvMpcFreq_Ly_range_new_reward
from simulator.pybullet.rl.rl_mpc_freq.envs.freq_env_tilted_ground_Ly_15 import DracoEnvMpcFreq_tilted_ground_Ly_15

if __name__ == "__main__":
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    load_path = os.path.join(
        cwd,
        'rl_model/freq_env/tilted_ground_Ly_15/PPO/redObsLy_15_tilted_10_downhill'
    )

    CURR_TIMESTEP = 5200000
    model_name = f'_TIME{CURR_TIMESTEP}.zip'
    norm_name = f'TIME{CURR_TIMESTEP}.pkl'
    norm_path = os.path.join(load_path, norm_name)
    load_path = os.path.join(load_path, model_name)

    reduced_obs_size = True
    mpc_freq = 5
    sim_dt = Config.CONTROLLER_DT

    env = DracoEnvMpcFreq_tilted_ground_Ly_15(
        mpc_freq,
        sim_dt,
        eval=[0, 0, 0],
        reduced_obs_size=reduced_obs_size,
        render=True)
    # video='freq_env_RL_Ly_range.mp4')
    monitor_env = Monitor(env)
    vec_env = DummyVecEnv([lambda: monitor_env])
    norm_env = VecNormalize.load(norm_path, vec_env)
    norm_env.training = False
    norm_env.norm_reward = False

    obs, info = env.reset()
    obs = norm_env.normalize_obs(obs)

    model = PPO.load(load_path, env=norm_env)
    counter = 0

    #Leg Width must be 0.1
    while True:
        counter += 1
        #action = torch.ones(AlipParams.N_BATCH,3)
        action, _ = model.predict(obs, deterministic=True)
        #action = 0*action
        config = read_config(cwd + '/config/draco/alip_command.ini')

        try:
            PARAMS = config['Parameters']
            Ly_des = PARAMS.getfloat('LY_DES')
            des_com_yaw = PARAMS.getfloat('COM_YAW')
            des_com_yaw = des_com_yaw * math.pi / 180

            Lx_offset = PARAMS.getfloat('LX_OFFSET')
        except KeyError:
            print("hey")

        ini_st_leg = np.random.choice([1, -1])
        env._set_command_policy_sim(Lx_offset, Ly_des, des_com_yaw, ini_st_leg)
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
