import os
import sys
import numpy as np
import datetime
import time
from math import pi

import gymnasium as gym

from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

from one_step_env import DracoEnvOneStepMpc
cwd = os.getcwd()
sys.path.append(cwd)

from config.draco.pybullet_simulation import Config

model_dir = cwd + "/rl_model/one_step/SAC"
#import tracemalloc
import argparse

import argparse

new_model = True
render = False

if __name__ == "__main__":
    if not new_model:
        parser = argparse.ArgumentParser(description='Training script for your RL model')
        parser.add_argument('--timesteps', type=str, help='FileToLoad')  # Default value is set as an example
        args = parser.parse_args()
        bash_timesteps = int(args.timesteps)

    yaw_max = 20
    Lx = 0.
    Ly = 0.
    randomized_command = False
    reduced_obs_size = True
    mpc_freq = 0
    sim_dt = Config.CONTROLLER_DT


    env = DracoEnvOneStepMpc(Lx, Ly, yaw_max, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = render)
    #env = VecNormalize(not_norm_env, norm_reward=False, clip_obs=50)

    learning_rate_ = 0.0003

    if reduced_obs_size: str1 = 'redOBS'
    else: str1 = 'fullOBS'
    if randomized_command: str2 = 'randCOMMAND'
    else: str2 = 'detCOMMAND'

    buffer_size_ = 1000000
    learning_starts_ = 100
    batch_size_ = 256

    #TODO: save replay buffer and add replay buffer

    save_dir = str1 + str2 + f"mpc_freq{mpc_freq}_SIMdt{sim_dt}_Lx_{Lx}_Ly_{Ly}_Yaw_{yaw_max}_new_start"         
    ## train model
    if new_model:
        tensorboard_dir = cwd + "/rl_log/one_step/sac/"
        #use MlpPolicy
        #"MultiInputPolicy"
        model = SAC("MlpPolicy", env, verbose=1, buffer_size = buffer_size_, learning_starts = learning_starts_, batch_size=batch_size_, tensorboard_log=tensorboard_dir, learning_rate=learning_rate_)#device = "cpu") #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]), 
        startTime = time.time()
        TIMESTEPS = 5000
        CURR_TIMESTEP = 0
    else:
        startTime = time.time()
        CURR_TIMESTEP = bash_timesteps    

        save_subdir = f"lstarts{learning_starts_}buffSize{buffer_size_}LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
        model_path = model_dir + '/' + save_dir + '/' + save_subdir
        print("model_path", model_path)
        model = PPO.load(model_path, env=env)
        TIMESTEPS =20*n_steps_



    while(True):
        try:
            model.learn(total_timesteps=TIMESTEPS, progress_bar=True, log_interval = 10, reset_num_timesteps=False, tb_log_name=save_dir)
            endTime = time.time()
            print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
            ## save the model
            CURR_TIMESTEP += TIMESTEPS
            save_subdir = f"lstarts{learning_starts_}buffSize{buffer_size_}LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
            save_path = model_dir + '/' + save_dir + '/' + save_subdir
            print(save_path)
            model.save(save_path)
            with open('timesteps_SAC.txt', 'w') as f:
                f.write(str(CURR_TIMESTEP))
                f.flush()
        except Exception as e:
            print(f"An error occurred during training: {e}")
            endTime = time.time()
            model = PPO.load(save_path, env = env)
