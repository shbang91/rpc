import pybullet as pb
import time
import os
from loop_rate_limiters import RateLimiter

cwd = os.getcwd()

import sys

sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  #include pybind module

import numpy as np

import signal
import shutil
import cv2

from config.optimo.pybullet_simulation import *
from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

def main():
    # Pybullet Renderer
    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    
    # Simulation Physics
    pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT, numSubSteps=1)
    pb.setGravity(0, 0, -9.81)
    
    # Sim parameters
    dt = Config.CONTROLLER_DT
    count = 0
    rate = 1 / dt
    
    
    
    # Load URDF
    optimo_arm = pb.loadURDF(cwd + "/robot_model/optimo/optimo.urdf",
                                [0, 0, 0.1],
                                [0, 0, 0, 1],
                                useFixedBase=True)
    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=True)
    
    # RUn Simulation
    while True:
        pb.stepSimulation()
        rate = RateLimiter(frequency=1. / dt)
    
    
if __name__ == "__main__":
    main()
    
    
