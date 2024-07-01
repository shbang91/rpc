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


from simulator.pybullet.rl.env import DracoEnv


if __name__ == "__main__":
    env = DracoEnv(render=True)

    from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    obs, info = env.reset()
    interface = info["interface"]

    model_dir = cwd + "/rl_model/PPO/redobs_yaw_10"

    model_path = f"{model_dir}/NSTEPS256_LEARNING_RATE0.0003_TIME128256"
    model = PPO.load(model_path, env=env)

    plot = False

    if plot == False:
        while True:
            #action = torch.ones(AlipParams.N_BATCH,3)
            action, _ = model.predict(obs, deterministic=False)
            print(action)
            obs, reward, done, trunc, info = env.step(action)
            print("reward", reward)
            print("reward info", info["reward_components"])
            if done:
                print(done)
                obs,info = env.reset()
    else:
        while not done:
            action, _ = model.predict(obs, deterministic=False)
            print(action)
            obs, reward, done, trunc, info = env.step(action)