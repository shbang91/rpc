import os
import sys
import numpy as np
import datetime
import time
from math import pi
import torch.nn as nn

import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize


cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from simulator.pybullet.rl.rl_one_step.envs.new_reward import DracoEnvOneStepMpc

from config.draco.pybullet_simulation import Config

model_dir = cwd + "/rl_model/one_step/optuna/PPO"
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

    assert sim_dt == Config.CONTROLLER_DT
    DEFAULT_HYPERPARAMS = {
        "policy": "MlpPolicy",
        "env": DracoEnvOneStepMpc(Lx_offset_des = Lx, Ly_des = Ly, yaw_des=yaw_max, mpc_freq=mpc_freq, sim_dt=sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render=False)
    }
    env = DracoEnvOneStepMpc(Lx, Ly, yaw_max, mpc_freq, sim_dt, randomized_command=randomized_command, reduced_obs_size=reduced_obs_size, render = render)
    #env = VecNormalize(not_norm_env, norm_reward=False, clip_obs=50)
    if reduced_obs_size: str1 = 'redOBS'
    else: str1 = 'fullOBS'
    if randomized_command: str2 = 'randCOMMAND'
    else: str2 = 'detCOMMAND'
    save_dir = str1 + str2 + f"mpc_freq{mpc_freq}_SIMdt{sim_dt}_Lx_{Lx}_Ly_{Ly}_Yaw_{yaw_max}_optuna_3"         




    "POLICY AND TRAINING PARAMS"
    #batch_size = 256 if batch_size > n_steps, batch_size = n_steps
    batch_size = 32
    n_steps = 32
    gamma = 0.98
    learning_rate = 8.234038460544177e-05
    ent_coef = 0.002069215758648058
    clip_range = 0.4
    n_epochs = 1
    gae_lambda = 0.9
    max_grad_norm = 2
    vf_coef = 0.3775973934546356
    net_arch_type = 'small'
    activation_fn_name = 'tanh'
    ortho_init = False

    net_arch = {
        "tiny": dict(pi=[64], vf=[64]),
        "small": dict(pi=[64, 64], vf=[64, 64]),
        "medium": dict(pi=[256, 256], vf=[256, 256]),
    }[net_arch_type]

    activation_fn = {"tanh": nn.Tanh, "relu": nn.ReLU, "elu": nn.ELU, "leaky_relu": nn.LeakyReLU}[activation_fn_name]
    tensorboard_dir = cwd + "/rl_log/one_step/ppo/optuna/"

    PPO_params = {
        "n_steps": n_steps,
        "batch_size": batch_size,
        "gamma": gamma,
        "learning_rate": learning_rate,
        "ent_coef": ent_coef,
        "clip_range": clip_range,
        "n_epochs": n_epochs,
        "gae_lambda": gae_lambda,
        "max_grad_norm": max_grad_norm,
        "vf_coef": vf_coef,
        # "sde_sample_freq": sde_sample_freq,
        "verbose":1,
        "tensorboard_log": tensorboard_dir,
        "policy_kwargs": dict(
            # log_std_init=log_std_init,
            net_arch=net_arch,
            activation_fn=activation_fn,
            ortho_init=ortho_init,
        ),
    }

    kwargs = DEFAULT_HYPERPARAMS.copy()
    # Sample hyperparameters.
    kwargs.update(PPO_params)


    ## train model
    if new_model:
        #use MlpPolicy
        #"MultiInputPolicy"
        model = PPO(**kwargs)
        startTime = time.time()
        TIMESTEPS = 10000
        CURR_TIMESTEP = 0
    else:
        startTime = time.time()
        CURR_TIMESTEP = bash_timesteps
        save_subdir = f"tuned_1_TIME{CURR_TIMESTEP}"
        model_path = model_dir + '/' + save_dir + '/' + save_subdir
        print("model_path", model_path)
        model = PPO.load(model_path, env=env)
        TIMESTEPS =10000



    while(True):
        try:
            model.learn(total_timesteps=TIMESTEPS, progress_bar=True, reset_num_timesteps=False, tb_log_name=save_dir)
            endTime = time.time()
            print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
            ## save the model
            CURR_TIMESTEP += TIMESTEPS
            save_subdir = f"tuned_1_TIME{CURR_TIMESTEP}"
            save_path = model_dir + '/' + save_dir + '/' + save_subdir
            print(save_path)
            model.save(save_path)
            with open('timesteps_train_optuna_1.txt', 'w') as f:
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
