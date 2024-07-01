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


from simulator.pybullet.rl.rl_one_step.envs.Ly_range import DracoEnvOneStepMpcRange
from simulator.pybullet.rl.rl_one_step.envs.Ly_range_zero import DracoEnvOneStepMpcRangeZero
if __name__ == "__main__":
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)
    path1 = '/home/carlos/Desktop/Austin/ONE_STEP_RESULTS'
    path2 = 'rl_model/Ly_range/PPO'
    file_name = 'redObsLy_range_new_r_second'
    file_name = 'redObsLy_range_new_reward_Lx_mod'
    load_path = os.path.join(path1, path2, file_name)
    CURR_TIMESTEP = 900000
    model_name = f'_TIME{CURR_TIMESTEP}.zip'
    norm_name = f'TIME{CURR_TIMESTEP}.pkl'
    norm_path = os.path.join(load_path, norm_name)
    load_path = os.path.join(load_path, model_name)

    reduced_obs_size = True
    mpc_freq = 0
    sim_dt = Config.CONTROLLER_DT

    env = DracoEnvOneStepMpcRange(mpc_freq, sim_dt, 
                                  reduced_obs_size=reduced_obs_size,  
                                  render = False,
                                  video = None)
    monitor_env = Monitor(env)
    vec_env = DummyVecEnv([lambda: monitor_env])
    norm_env = VecNormalize.load(norm_path, vec_env)
    norm_env.training = False
    norm_env.norm_reward = False


    env_zero = DracoEnvOneStepMpcRangeZero(mpc_freq, sim_dt, 
                                    reduced_obs_size=reduced_obs_size,  
                                    render = False,
                                    video = None)
    mon_env_zero = Monitor(env_zero)

    model = PPO.load(load_path, env=norm_env)
    
    print("eval1", evaluate_policy(model, norm_env, n_eval_episodes=25, deterministic = True, render = False, warn = True, return_episode_rewards=True))
    print("zero", evaluate_policy(model, env_zero, n_eval_episodes=25, deterministic = True, render = False, warn = True,return_episode_rewards=True))