import os
import sys
import numpy as np
import datetime
import time
from math import pi

import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from simulator.pybullet.rl.rl_one_step.envs.new_reward import DracoEnvOneStepMpc
from simulator.pybullet.rl.schedule_callback import *
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, EvalCallback
from stable_baselines3.common.callbacks import BaseCallback

from config.draco.pybullet_simulation import Config

model_dir = cwd + "/rl_model/one_step/PPO"
env_dir = cwd + "/rl_env/one_step/PPO"
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
    reduced_obs_size = False
    mpc_freq = 0
    sim_dt = 0.00175

    render = False
    env = DracoEnvOneStepMpc(Lx, Ly, yaw_max, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = render)

    monitor_env = Monitor(env)
    vec_env = DummyVecEnv([lambda: monitor_env])

    n_steps_ = 256 #512
    batch_size_ = 64
    learning_rate_ = 0.0003 

    if reduced_obs_size: str1 = 'redOBS'
    else: str1 = 'fullOBS'
    if randomized_command: str2 = 'randCOMMAND'
    else: str2 = 'detCOMMAND'

    eval_env = DracoEnvOneStepMpc(Lx, Ly, yaw_max, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = render)
    eval_monitor_env = Monitor(eval_env)
    eval_vec_env = DummyVecEnv([lambda: eval_monitor_env])
    norm_eval_env = VecNormalize(eval_vec_env,norm_obs = True, norm_reward = False, clip_obs = 60, gamma = 0.99)

    lr_callback = CallbackHyperparamsSchedulePOO_mean_len()
    eval_callback = EvalCallback(norm_eval_env, eval_freq=3*n_steps_, deterministic=True, render = False, )
    callback = CallbackList([lr_callback, eval_callback])

    save_dir = str1 + str2 + f"mpc_freq{mpc_freq}_SIMdt{sim_dt}Yaw_{yaw_max}_lr_linear_callback_2" 
    save_path = os.path.join(model_dir, save_dir)      
    ## train model
    if new_model:
        tensorboard_dir = cwd + "/rl_log/one_step/ppo/optuna/"
        #use MlpPolicy
        #"MultiInputPolicy"
        norm_env = VecNormalize(vec_env, norm_obs = True, norm_reward = False, clip_obs = 60, gamma = 0.99)

        model = PPO("MlpPolicy", norm_env, verbose=1, n_steps = n_steps_, batch_size=batch_size_, tensorboard_log=tensorboard_dir, learning_rate=learning_rate_) #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]), 
        startTime = time.time()
        TIMESTEPS = 10000
        CURR_TIMESTEP = 0
    else:
        startTime = time.time()
        CURR_TIMESTEP = bash_timesteps

        load_dir = str1 + str2 + f"mpc_freq{mpc_freq}_SIMdt{sim_dt}Yaw_{yaw_max}_lr_linear_callback_2" 
        load_path = os.path.join(model_dir, load_dir)
        load_subdir = f"NSTEPS{n_steps_}_TIME{CURR_TIMESTEP}"
        model_path = os.path.join(load_path, load_subdir)
        env_file = f"TIME{CURR_TIMESTEP}.pkl"
        load_env_path = os.path.join(load_path, env_file)
        norm_env = VecNormalize.load(load_env_path, vec_env)

        model = PPO.load(model_path, env=norm_env)

        TIMESTEPS =10000




    while(True):
        try:
            model.learn(total_timesteps=TIMESTEPS, progress_bar=True, reset_num_timesteps=False, tb_log_name=save_dir, callback=callback)
            endTime = time.time()
            print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
            ## save the model
            CURR_TIMESTEP += TIMESTEPS
            save_subdir = f"NSTEPS{n_steps_}_TIME{CURR_TIMESTEP}"
            model_path = save_path + '/' + save_subdir
            env_file = f"TIME{CURR_TIMESTEP}.pkl"
            save_env_path = os.path.join(save_path, env_file)
            print(save_path)
            model.save(model_path)
            print(save_env_path)
            norm_env.save(save_env_path)
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
