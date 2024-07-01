import os
import sys
import numpy as np
import datetime
import time
from math import pi
import torch

import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from simulator.pybullet.rl.rl_one_step.envs.new_reward import DracoEnvOneStepMpc

from config.draco.pybullet_simulation import Config

model_dir = cwd + "/rl_model/one_step/PPO"
#import tracemalloc
import argparse

import argparse

new_model = True

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
    sim_dt = 0.00175

    render = False
    env = DracoEnvOneStepMpc(Lx, Ly, yaw_max, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = render)
    #env = VecNormalize(not_norm_env, norm_reward=False, clip_obs=50)

    n_steps_ = 256 #512
    batch_size_ = 64
    learning_rate_ = 0.0003

    if reduced_obs_size: str1 = 'redOBS'
    else: str1 = 'fullOBS'
    if randomized_command: str2 = 'randCOMMAND'
    else: str2 = 'detCOMMAND'

    


    save_dir = str1 + str2 + f"mpc_freq{mpc_freq}_SIMdt{sim_dt}_Yaw_{yaw_max}_last_0"         
    ## train model
    if new_model:
        tensorboard_dir = cwd + "/rl_log/one_step/ppo/optuna/"
        #use MlpPolicy
        #"MultiInputPolicy"
        model = PPO("MlpPolicy", env, verbose=1, n_steps = n_steps_, batch_size=batch_size_, tensorboard_log=tensorboard_dir, learning_rate=learning_rate_, device = "cpu") #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]),
        a = model.get_parameters()
        a['policy']['action_net.weight'] = torch.zeros_like(a['policy']['action_net.weight'])
        a['policy']['action_net.bias'] = torch.zeros_like(a['policy']['action_net.bias'])
        model.set_parameters(a)
        
        startTime = time.time()
        TIMESTEPS = 10000
        CURR_TIMESTEP = 0
    else:
        startTime = time.time()
        CURR_TIMESTEP = bash_timesteps
        save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
        model_path = model_dir + '/' + save_dir + '/' + save_subdir
        print("model_path", model_path)
        model = PPO.load(model_path, env=env)
        TIMESTEPS =20*n_steps_



    while(True):
        try:
            model.learn(total_timesteps=TIMESTEPS, progress_bar=True, reset_num_timesteps=False, tb_log_name=save_dir)
            endTime = time.time()
            print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
            ## save the model
            CURR_TIMESTEP += TIMESTEPS
            save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
            save_path = model_dir + '/' + save_dir + '/' + save_subdir
            print(save_path)
            model.save(save_path)
            with open('timesteps_2.txt', 'w') as f:
                f.write(str(CURR_TIMESTEP))
                f.flush()
        except Exception as e:
            print(f"An error occurred during training: {e}")
            endTime = time.time()
            #except ValueError:
            # close the progress bar
            #callbacks[1].on_training_end()
            #pbar_callback = ProgressBarCallback()
            #model = PPO.load(save_path, env = env)
