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
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, EvalCallback

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from simulator.pybullet.rl.rl_one_step.envs.burn_in import DracoEnvOneStepMpc as Burn_in_Env
from simulator.pybullet.rl.rl_one_step.envs.new_reward import DracoEnvOneStepMpc 



from config.draco.pybullet_simulation import Config

model_dir = cwd + "/rl_model/one_step/PPO"
env_dir = cwd + "/rl_env/one_step/PPO"
#import tracemalloc
import argparse

import argparse

new_model = True
burn_in = False

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


    if (burn_in):
        env = Burn_in_Env(Lx, Ly, yaw_max, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = render)

        monitor_env = Monitor(env)
        vec_env = DummyVecEnv([lambda: monitor_env])
    else: 
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

    eval_callback = EvalCallback(norm_eval_env, eval_freq=3*n_steps_, deterministic=True, render = False, )

    save_dir = str1 + str2 + f"Yaw_{yaw_max}_burn_in" 
    save_path = os.path.join(model_dir, save_dir)      
    ## train model

    if burn_in:
        tensorboard_dir = cwd + "/rl_log/one_step/ppo/optuna/"

        norm_env = VecNormalize(vec_env, norm_obs = True, norm_reward = False, clip_obs = 60, gamma = 0.99)
        save_path += 'st'
        model = PPO("MlpPolicy", norm_env, verbose=1, n_steps = n_steps_, batch_size=batch_size_, tensorboard_log=tensorboard_dir, learning_rate=learning_rate_, device = 'cpu') #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]), 
        startTime = time.time()


        TIMESTEPS = 3*n_steps_
        CURR_TIMESTEP = 0

        save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
        model_path = save_path + '/' + save_subdir
        model.save(model_path)
    elif new_model:
        tensorboard_dir = cwd + "/rl_log/one_step/ppo/optuna/"


        norm_env = VecNormalize(vec_env, norm_obs = True, norm_reward = False, clip_obs = 60, gamma = 0.99)

        model = PPO("MlpPolicy", norm_env, verbose=1, n_steps = n_steps_, batch_size=batch_size_, tensorboard_log=tensorboard_dir, learning_rate=learning_rate_, device = 'cpu') #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]), 
        model_par = model.get_parameters()

        save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{3840}"

        burn_in_path = save_path + '/' + save_subdir
        burn_in_model = PPO.load(burn_in_path, env=norm_env)

        a = burn_in_model.get_parameters()


        model_par['policy']['mlp_extractor.value_net.0.weight'] = a['policy']['mlp_extractor.value_net.0.weight']
        model_par['policy']['mlp_extractor.value_net.0.bias'] = a['policy']['mlp_extractor.value_net.0.bias']
        model_par['policy']['mlp_extractor.value_net.2.weight'] = a['policy']['mlp_extractor.value_net.2.weight']
        model_par['policy']['mlp_extractor.value_net.2.bias'] = a['policy']['mlp_extractor.value_net.2.bias']
        model_par['policy']['value_net.weight'] = a['policy']['value_net.weight']
        model_par['policy']['value_net.bias'] = a['policy']['value_net.bias']

        model_par['policy']['action_net.weight'] = torch.zeros_like(a['policy']['action_net.weight'])
        model_par['policy']['action_net.bias'] = torch.zeros_like(a['policy']['action_net.bias'])

        model.set_parameters(model_par)
        
        startTime = time.time()


        TIMESTEPS = 10000
        CURR_TIMESTEP = 0

        save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
        model_path = save_path + '/' + save_subdir
        model.save(model_path)
    else: 
        startTime = time.time()
        CURR_TIMESTEP = bash_timesteps

        save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
        model_path = os.path.join(save_path, save_subdir)
        env_file = f"TIME{CURR_TIMESTEP}.pkl"
        save_env_path = os.path.join(save_path, env_file)
        norm_env = VecNormalize.load(save_env_path, vec_env)

        model = PPO.load(model_path, env=norm_env)

        TIMESTEPS =10000



    while(True):
        try:
            model.learn(total_timesteps=TIMESTEPS, progress_bar=True, reset_num_timesteps=False, tb_log_name=save_dir, callback=eval_callback)
            endTime = time.time()
            print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
            ## save the model
            CURR_TIMESTEP += TIMESTEPS
            save_subdir = f"NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
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
