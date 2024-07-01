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
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, EvalCallback

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from simulator.pybullet.rl.rl_mpc_freq.envs.freq_env_turn_30 import DracoEnvMpcFreq_turn_30


n_steps_ = 8192 #256
batch_size_ = 1024
learning_rate_ = 0.0003
mpc_freq = 5
sim_dt = 0.00175
reduced_obs_size = True

render = False
env = DracoEnvMpcFreq_turn_30(mpc_freq, sim_dt, reduced_obs_size=reduced_obs_size, render = render)

monitor_env = Monitor(env)
vec_env = DummyVecEnv([lambda: monitor_env])



policy_kwargs = { 'full_std': False}
norm_env = VecNormalize(vec_env, norm_obs = True, norm_reward = False, clip_obs = 60, gamma = 0.99)

model = PPO("MlpPolicy", norm_env, verbose=1, 
            n_steps = n_steps_, 
            batch_size=batch_size_, 
            learning_rate=learning_rate_,
            policy_kwargs=policy_kwargs,
            use_sde=True,
            device='cpu') #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]),


b = model.get_parameters()
print(b['policy']['action_net.bias'])
print(b['policy']['action_net.weight'])

print("hey")
for i in b:
    print(b[i])

print("END")
startTime = time.time()
TIMESTEPS = 200000
CURR_TIMESTEP = 0

model2 = PPO("MlpPolicy", norm_env, verbose=1, 
            n_steps = n_steps_, 
            batch_size=batch_size_, 
            learning_rate=learning_rate_,
            policy_kwargs=policy_kwargs,
            use_sde=True,
            device='cpu') #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]),

c = model.get_parameters()
print(c['policy'])
