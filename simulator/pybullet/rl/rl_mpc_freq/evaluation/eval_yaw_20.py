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


from simulator.pybullet.rl.rl_mpc_freq.envs.freq_env_turn_20_v3 import DracoEnvMpcFreq_turn_20_v3

if __name__ == "__main__":
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)
    load_path = os.path.join(cwd, 'rl_model/freq_env/Ly_10/PPO/redObsyaw_20_v3_n_steps_32768_batch_2048')
    load_path = '/home/carlos/Desktop/Austin/FREQ_RESULTS/freq_env/rl_model/PPO/redObsyaw_20_v3_n_steps_32768_batch_2048'

   
    CURR_TIMESTEP = 19000000
    model_name = f'_TIME{CURR_TIMESTEP}.zip'
    norm_name = f'TIME{CURR_TIMESTEP}.pkl'
    norm_path = os.path.join(load_path, norm_name)
    load_path = os.path.join(load_path, model_name)
    reduced_obs_size = True
    mpc_freq = 5
    sim_dt = Config.CONTROLLER_DT

    yaw = math.pi*20/180
    env = DracoEnvMpcFreq_turn_20_v3(mpc_freq, sim_dt, eval = [0,0,yaw] ,reduced_obs_size=reduced_obs_size, render = True,
                                     video = 'mpc_freq_RL_yaw_20_v2.mp4')
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
        #action = 0*action
        config = read_config('/home/carlos/Desktop/Austin/SeungHyeonProject/rpc/config/draco/alip_command.ini')
        

        
        ini_st_leg = np.random.choice([1, -1])  
        #print("action: ",         env._normalise_action(action))
        obs, reward, done, trunc, info = env.step(action)
        obs = norm_env.normalize_obs(obs)
        #print("reward", reward)
        #print("reward info", info["reward_components"])
        if done:
            #print(done)
            obs,info = env.reset()
            #print(obs)
            obs = norm_env.normalize_obs(obs)
            #print(obs)
            
        """
        if counter > 20*32:
            counter = 0
            obs,info = env.reset()
            obs = norm_env.normalize_obs(obs)
        """