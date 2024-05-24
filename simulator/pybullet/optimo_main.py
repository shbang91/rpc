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

import optimo_interface_py

def get_sensor_data_from_pybullet(robot):
    
    # Initialize sensor data
    joint_pos, joint_vel = np.zeros(7), np.zeros(7)
 
    ## TODO: Get Joint SEA Torque
    
    # Joint Position
    joint_pos[0] = pb.getJointState(robot, OptimoJointIdx.joint1)[0]
    joint_pos[1] = pb.getJointState(robot, OptimoJointIdx.joint2)[0]
    joint_pos[2] = pb.getJointState(robot, OptimoJointIdx.joint3)[0]
    joint_pos[3] = pb.getJointState(robot, OptimoJointIdx.joint4)[0]
    joint_pos[4] = pb.getJointState(robot, OptimoJointIdx.joint5)[0]
    joint_pos[5] = pb.getJointState(robot, OptimoJointIdx.joint6)[0]
    joint_pos[6] = pb.getJointState(robot, OptimoJointIdx.joint7)[0]
    
    # Joint Velocity
    joint_vel[0] = pb.getJointState(robot, OptimoJointIdx.joint1)[1]
    joint_vel[1] = pb.getJointState(robot, OptimoJointIdx.joint2)[1]
    joint_vel[2] = pb.getJointState(robot, OptimoJointIdx.joint3)[1]
    joint_vel[3] = pb.getJointState(robot, OptimoJointIdx.joint4)[1]
    joint_vel[4] = pb.getJointState(robot, OptimoJointIdx.joint5)[1]
    joint_vel[5] = pb.getJointState(robot, OptimoJointIdx.joint6)[1]
    
def apply_control_inpit_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL
    
    # Apply Torque Control (Emulating current applied to motors)
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint1, controlMode=mode, force=command[0])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint2, controlMode=mode, force=command[1])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint3, controlMode=mode, force=command[2])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint4, controlMode=mode, force=command[3])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint5, controlMode=mode, force=command[4])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint6, controlMode=mode, force=command[5])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint7, controlMode=mode, force=command[6])
    



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
    
    
