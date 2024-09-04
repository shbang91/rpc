import pybullet as pb
import time
import os
import sys
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

import signal
import shutil
import cv2

if __name__ == "__main__":
    ## connect pybullet sim server
    pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    pb.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=120,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.3],
    )
    ## sim physics setting
    pb.setPhysicsEngineParameter(fixedTimeStep=0.001, numSubSteps=1)
    pb.setGravity(0, 0, -9.81)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    robot = pb.loadURDF(
        cwd + "/robot_model/go2/go2_description.urdf",
        [0., 0., 0.45],
        [0, 0, 0, 1],
        useFixedBase=1,
    )

    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf",
                         useFixedBase=1)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    while True:
        pb.stepSimulation()  # step simulation
