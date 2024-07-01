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


from simulator.pybullet.rl.rl_one_step.envs.dist_env_Ly_10_test import DracoEnvOneStepMpc_Ly_10_dist_diag

if __name__ == "__main__":
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    load_path = '/home/carlos/Desktop/Austin/ONE_STEP_RESULTS/rl_model/Ly_range/PPO/redObsLy_10_dist'
    #load_path = os.path.join(cwd, 'rl_model/freq_env/Ly_10/PPO/redObsLy_10_disturbance_full')
    CURR_TIMESTEP = 430000
    model_name = f'_TIME{CURR_TIMESTEP}.zip'
    norm_name = f'TIME{CURR_TIMESTEP}.pkl'
    norm_path = os.path.join(load_path, norm_name)
    load_path = os.path.join(load_path, model_name)

    reduced_obs_size = True
    mpc_freq = 0
    sim_dt = Config.CONTROLLER_DT

    env = DracoEnvOneStepMpc_Ly_10_dist_diag(mpc_freq, 
                                sim_dt, 
                                eval = [0, 10, 0], 
                                reduced_obs_size=reduced_obs_size, 
                                render = True,
                                disturbance = True)
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
        counter+=1
        #action = torch.ones(AlipParams.N_BATCH,3)
        action, _ = model.predict(obs, deterministic=True)
        action = 0*action

        ini_st_leg = np.random.choice([1, -1])  
        # env._set_command_policy_sim(0, 10, 0, ini_st_leg)
        env._set_command_policy_sim(0, 10, 0, ini_st_leg)

        #print("action: ",         env._normalise_action(action))

        obs, reward, done, trunc, info = env.step(action)
        #print(obs)
        obs = norm_env.normalize_obs(obs)
        #print("reward", reward)
        #print("reward info", info["reward_components"])
        if done:
            print(done)
            obs,info = env.reset()
            obs = norm_env.normalize_obs(obs)
        """
        if counter > 20*32:
            counter = 0
            obs,info = env.reset()
            obs = norm_env.normalize_obs(obs)
        """