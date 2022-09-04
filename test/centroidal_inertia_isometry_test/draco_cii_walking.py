import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import copy
import time

import pybullet as pb
import numpy as np

from util.python_utils import pybullet_util
from util.python_utils.pinocchio_robot_system import PinocchioRobotSystem

from plot.data_saver import DataSaver

## parameters
INITIAL_POS = [0., 0., 1.]
INITIAL_QUAT = [0., 0., 0., 1.]

if __name__ == 'main':
    #pybullet GUI for visualization
    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=2.0,
                                  cameraYaw=120,
                                  cameraPitch=-15,
                                  cameraTargetPosition=[0, 0, 1.])

    ## spawn draco in pybullet
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    draco = pb.loadURDF(cwd + '/robot_model/draco/draco_modified.urdf',
                        INITIAL_POS,
                        INITIAL_QUAT,
                        useFixedBase=True)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)


    ## robot config
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        draco, INITIAL_POS, INITIAL_QUAT, False)
    nominal_sensor_data = pybullet_util.get_sensor_data(
        draco, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)
    joint_pos = copy.deepcopy(nominal_sensor_data['joint_pos'])

    ## update virtual robot model
    robot_system = PinocchioRobotSystem(
        cwd + '/robot_model/draco/draco_modified.urdf',
        cwd + '/robot_model/draco', True, False)
    robot_system.update_system(nominal_sensor_data['base_joint_pos'],
                               nominal_sensor_data['base_joint_quat'],
                               nominal_sensor_data['base_joint_lin_vel'],
                               nominal_sensor_data['base_joint_ang_vel'],
                               joint_pos, nominal_sensor_data['joint_vel'],
                               True)

    ## calculate rotational CCRBI 
    nominal_inertia = robot_system.Ig[0:3, 0:3]

    ## data saver for RCCRBI
    data_saver = DataSaver('draco_cii_proximal_walking_traj.pkl')

    ## create operational space walking trajectories with parameters
    ## parameters: Foot SE(3), Swing height, Swing time, number of node

    ## create swing foot trajectories



