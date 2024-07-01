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
from stable_baselines3.common.evaluation import evaluate_policy
from util.python_utils.util import read_config


from simulator.pybullet.rl.rl_one_step.envs.turn_20 import DracoEnvOneStepMpcYaw_20
from simulator.pybullet.rl.rl_mpc_freq.envs.freq_env_turn_20_v3 import DracoEnvMpcFreq_turn_20_v3
if __name__ == "__main__":
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)
    one_step_path = '/home/carlos/Desktop/Austin/ONE_STEP_RESULTS/rl_model/Ly_range/PPO/redObsLy_turn_20_third/'
    freq_path = '/home/carlos/Desktop/Austin/FREQ_RESULTS/freq_env/rl_model/PPO/redObsyaw_20_v3_n_steps_32768_batch_2048'

    CURR_TIMESTEP_freq = 19000000
    CURR_TIMESTEP_one = 340000
    model_name_one = f'_TIME{CURR_TIMESTEP_one}.zip'
    norm_name_one = f'TIME{CURR_TIMESTEP_one}.pkl'
    norm_path_one = os.path.join(one_step_path, norm_name_one)
    load_path_one = os.path.join(one_step_path, model_name_one)

    model_name_freq = f'_TIME{CURR_TIMESTEP_freq}.zip'
    norm_name_freq = f'TIME{CURR_TIMESTEP_freq}.pkl'
    norm_path_freq = os.path.join(freq_path, norm_name_freq)
    load_path_freq = os.path.join(freq_path, model_name_freq)


    sim_dt = Config.CONTROLLER_DT

    env_one_zero = DracoEnvOneStepMpcYaw_20(0, sim_dt, 
                                  reduced_obs_size=True,  
                                  render = False,
                                  video = None,
                                  zero = True)
    monitor_env_one_zero = Monitor(env_one_zero)


    env_one = DracoEnvOneStepMpcYaw_20(0, sim_dt, 
                                  reduced_obs_size=True,  
                                  render = False,
                                  video = None)
    monitor_env_one = Monitor(env_one)
    vec_env_one = DummyVecEnv([lambda: monitor_env_one])
    norm_env_one = VecNormalize.load(norm_path_one, vec_env_one)
    norm_env_one.training = False
    norm_env_one.norm_reward = False



    model_one = PPO.load(load_path_one, env=norm_env_one)
    print("start")
    #print("one eval", evaluate_policy(model_one, norm_env_one, n_eval_episodes=100, deterministic = True, render = False, warn = True, return_episode_rewards=True))
    #print("one zero", evaluate_policy(model_one, monitor_env_one_zero, n_eval_episodes=100, deterministic = True, render = False, warn = True,return_episode_rewards=True))




    env_freq_zero = DracoEnvMpcFreq_turn_20_v3(5, sim_dt, 
                                        reduced_obs_size=True,  
                                        render = False,
                                        video = None,
                                        zero = True)
    monitor_env_freq_zero = Monitor(env_freq_zero)

    env_freq = DracoEnvMpcFreq_turn_20_v3(5, sim_dt, 
                                          reduced_obs_size=True,  
                                          render = False,
                                          video = None)
    monitor_env_freq = Monitor(env_freq)

    vec_env_freq = DummyVecEnv([lambda: monitor_env_freq])
    norm_env_freq = VecNormalize.load(norm_path_freq, vec_env_freq)
    norm_env_freq.training = False
    norm_env_freq.norm_reward = False

    model_freq = PPO.load(load_path_freq, env=norm_env_freq)

    print("freq eval", evaluate_policy(model_freq, norm_env_freq, n_eval_episodes=100, deterministic = True, render = False, warn = True, return_episode_rewards=True))
    print("freq zero", evaluate_policy(model_freq, monitor_env_freq_zero, n_eval_episodes=100, deterministic = True, render = False, warn = True,return_episode_rewards=True))



