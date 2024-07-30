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
    joint_pos, joint_vel = np.zeros(7+9), np.zeros(7+9)
 
    ## TODO: Get Joint SEA Torque
    
    ## TODO: Use for loop istead?
    # Joint Position (Arm)
    joint_pos[0] = pb.getJointState(robot, OptimoJointIdx.joint1)[0]
    joint_pos[1] = pb.getJointState(robot, OptimoJointIdx.joint2)[0]
    joint_pos[2] = pb.getJointState(robot, OptimoJointIdx.joint3)[0]
    joint_pos[3] = pb.getJointState(robot, OptimoJointIdx.joint4)[0]
    joint_pos[4] = pb.getJointState(robot, OptimoJointIdx.joint5)[0]
    joint_pos[5] = pb.getJointState(robot, OptimoJointIdx.joint6)[0]
    joint_pos[6] = pb.getJointState(robot, OptimoJointIdx.joint7)[0]
    
    # Joint Position (Hand)
    joint_pos[7] = pb.getJointState(robot, PlatoJointIdx.joint1)[0]
    joint_pos[8] = pb.getJointState(robot, PlatoJointIdx.joint2)[0]
    joint_pos[9] = pb.getJointState(robot, PlatoJointIdx.joint3)[0]
    joint_pos[10] = pb.getJointState(robot, PlatoJointIdx.joint4)[0]
    joint_pos[11] = pb.getJointState(robot, PlatoJointIdx.joint5)[0]
    joint_pos[12] = pb.getJointState(robot, PlatoJointIdx.joint6)[0]
    joint_pos[13] = pb.getJointState(robot, PlatoJointIdx.joint7)[0]
    joint_pos[14] = pb.getJointState(robot, PlatoJointIdx.joint8)[0]
    joint_pos[15] = pb.getJointState(robot, PlatoJointIdx.joint9)[0]
    
    # Joint Velocity (Arm)
    joint_vel[0] = pb.getJointState(robot, OptimoJointIdx.joint1)[1]
    joint_vel[1] = pb.getJointState(robot, OptimoJointIdx.joint2)[1]
    joint_vel[2] = pb.getJointState(robot, OptimoJointIdx.joint3)[1]
    joint_vel[3] = pb.getJointState(robot, OptimoJointIdx.joint4)[1]
    joint_vel[4] = pb.getJointState(robot, OptimoJointIdx.joint5)[1]
    joint_vel[5] = pb.getJointState(robot, OptimoJointIdx.joint6)[1]
    joint_vel[6] = pb.getJointState(robot, OptimoJointIdx.joint7)[1]
    
    # Joint Velocity (Hand)
    joint_vel[7] = pb.getJointState(robot, PlatoJointIdx.joint1)[1]
    joint_vel[8] = pb.getJointState(robot, PlatoJointIdx.joint2)[1]
    joint_vel[9] = pb.getJointState(robot, PlatoJointIdx.joint3)[1]
    joint_vel[10] = pb.getJointState(robot, PlatoJointIdx.joint4)[1]
    joint_vel[11] = pb.getJointState(robot, PlatoJointIdx.joint5)[1]
    joint_vel[12] = pb.getJointState(robot, PlatoJointIdx.joint6)[1]
    joint_vel[13] = pb.getJointState(robot, PlatoJointIdx.joint7)[1]
    joint_vel[14] = pb.getJointState(robot, PlatoJointIdx.joint8)[1]
    joint_vel[15] = pb.getJointState(robot, PlatoJointIdx.joint9)[1]
    
    
    return joint_pos, joint_vel
    
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
    
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint1, controlMode=mode, force=command[7])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint2, controlMode=mode, force=command[8])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint3, controlMode=mode, force=command[9])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint4, controlMode=mode, force=command[10])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint5, controlMode=mode, force=command[11])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint6, controlMode=mode, force=command[12])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint7, controlMode=mode, force=command[13])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint8, controlMode=mode, force=command[14])
    pb.setJointMotorControl2(robot, PlatoJointIdx.joint9, controlMode=mode, force=command[15])
    
def set_init_config_pybullet(robot):
    pb.resetJointState(robot, OptimoJointIdx.joint1, 0.0, 0.0)
    pb.resetJointState(robot, OptimoJointIdx.joint2, Config.INITIAL_JOINT_POSITION[1], 0.0)
    pb.resetJointState(robot, OptimoJointIdx.joint3, Config.INITIAL_JOINT_POSITION[2], 0.0)
    pb.resetJointState(robot, OptimoJointIdx.joint4, Config.INITIAL_JOINT_POSITION[3], 0.0)
    pb.resetJointState(robot, OptimoJointIdx.joint5, Config.INITIAL_JOINT_POSITION[4], 0.0)
    pb.resetJointState(robot, OptimoJointIdx.joint6, Config.INITIAL_JOINT_POSITION[5], 0.0)
    pb.resetJointState(robot, OptimoJointIdx.joint7, Config.INITIAL_JOINT_POSITION[6], 0.0)
    
    pb.resetJointState(robot, PlatoJointIdx.joint1, Config.INITIAL_JOINT_POSITION[7], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint2, Config.INITIAL_JOINT_POSITION[8], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint3, Config.INITIAL_JOINT_POSITION[9], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint4, Config.INITIAL_JOINT_POSITION[10], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint5, Config.INITIAL_JOINT_POSITION[11], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint6, Config.INITIAL_JOINT_POSITION[12], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint7, Config.INITIAL_JOINT_POSITION[13], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint8, Config.INITIAL_JOINT_POSITION[14], 0.0)
    pb.resetJointState(robot, PlatoJointIdx.joint9, Config.INITIAL_JOINT_POSITION[15], 0.0)
    print("Initial Configuration Set")
    
from scipy.spatial.transform import Rotation as R
def add_tf_visualization(robot, link_index):
    """
    Add a visualization of the desired frame of the robot. 
    """
    pos, ori = pb.getLinkState(robot, link_index)[0:2]

    # Define the axis unit vectors
    x_axis = np.array([0.1, 0, 0])
    y_axis = np.array([0, 0.1, 0])
    z_axis = np.array([0, 0, 0.1])

    # Convert the quaternion to a rotation matrix
    rot = R.from_quat(ori).as_matrix()

    # Rotate the axis vectors by the orientation
    x_ending = pos + rot.dot(x_axis)
    y_ending = pos + rot.dot(y_axis)
    z_ending = pos + rot.dot(z_axis)

    # Define the color of the debug line 
    red = [1, 0, 0]  # RGB values for red
    blue = [0, 0, 1]  # RGB values for blue
    green = [0, 1, 0]  # RGB values for green

    pb.addUserDebugLine(pos, x_ending, red, lineWidth=4, lifeTime=0)
    pb.addUserDebugLine(pos, y_ending, green, lineWidth=4, lifeTime=0)
    pb.addUserDebugLine(pos, z_ending, blue, lineWidth=4, lifeTime=0)

def apply_control_input_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL
    
    # Optimo Arm
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint1, controlMode=mode, force=command[0])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint2, controlMode=mode, force=command[1])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint3, controlMode=mode, force=command[2])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint4, controlMode=mode, force=command[3])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint5, controlMode=mode, force=command[4])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint6, controlMode=mode, force=command[5])
    pb.setJointMotorControl2(robot, OptimoJointIdx.joint7, controlMode=mode, force=command[6])

    # Plato Hand
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint1, controlMode=mode, force=command[7])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint2, controlMode=mode, force=command[8])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint3, controlMode=mode, force=command[9])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint4, controlMode=mode, force=command[10])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint5, controlMode=mode, force=command[11])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint6, controlMode=mode, force=command[12])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint7, controlMode=mode, force=command[13])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint8, controlMode=mode, force=command[14])
    # pb.setJointMotorControl2(robot, PlatoJointIdx.joint9, controlMode=mode, force=command[15])
    

def main():
    # Pybullet Renderer
    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    
    # Simulation Physics
    pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT, numSubSteps=1)
    pb.setGravity(0, 0, -9.81)
    
    ## robot spawn & initial kinematics and dynamics setting
    
    # Sim parameters
    dt = Config.CONTROLLER_DT
    count = 0
    rate = 1 / dt
    
    # Load URDF
    robot = pb.loadURDF(cwd + "/robot_model/optimo/optimo.urdf",
                                [0, 0, 0],
                                [0, 0, 0, 1],
                                useFixedBase=True)
    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=True)
    
    
    # Initial joint configuration
    set_init_config_pybullet(robot)
    
    # Add End-effector Visualization for initial configuration
    add_tf_visualization(robot, OptimoLinkIdx.ee)
    add_tf_visualization(robot, PlatoLinkIdx.ee1)
    add_tf_visualization(robot, PlatoLinkIdx.ee2)
    add_tf_visualization(robot, PlatoLinkIdx.ee3)
    
    n_q, n_v, n_a, joint_id_dict, link_id_dict, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
    robot, Config.INITIAL_BASE_JOINT_POS,
    Config.INITIAL_BASE_JOINT_QUAT, Config.PRINT_ROBOT_INFO)

    # Joint Friction and Damping
    pybullet_util.set_joint_friction(robot, joint_id_dict, 0)
    pybullet_util.set_link_damping(robot, link_id_dict, 0., 0.)

    # Initialize RPC Interface
    rpc_optimo_interface = optimo_interface_py.OptimoInterface()
    rpc_optimo_command = optimo_interface_py.OptimoCommand()
    rpc_optimo_sensor_data = optimo_interface_py.OptimoSensorData()
    
    
    # Simulation Time Parameters
    rate = RateLimiter(frequency=1. / dt)
    
    #  Simulation Main Loop
    while True:
        
        
        # get sensor data from pybullet
        joint_pos, joint_vel = get_sensor_data_from_pybullet(robot)
        
        # copy sensor data to rpc_optimo_sensor_data
        rpc_optimo_sensor_data.joint_pos_ = joint_pos
        rpc_optimo_sensor_data.joint_vel_ = joint_vel
        
        rpc_optimo_interface.GetCommand(rpc_optimo_sensor_data, rpc_optimo_command)
        
        rpc_trq_command = rpc_optimo_command.joint_trq_cmd_
        rpc_joint_pos_command = rpc_optimo_command.joint_pos_cmd_
        rpc_joint_vel_command = rpc_optimo_command.joint_vel_cmd_
        

        # print("Command Torque: ", rpc_trq_command, end="\r")
        apply_control_input_to_pybullet(robot, rpc_trq_command)
        
        
        pb.stepSimulation()
        # print without newline
        # print("Simulation Time: ", count * dt, end="\r")

        
        
    
        
        count += 1    
        rate.sleep()
        
            
    
    
if __name__ == "__main__":
    main()
    
    
