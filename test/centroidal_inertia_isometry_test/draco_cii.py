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

INITIAL_POS = [0., 0., 1.]
INITIAL_QUAT = [0., 0., 0., 1.]

if __name__ == '__main__':
    #pybullet env
    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=2.0,
                                  cameraYaw=120,
                                  cameraPitch=-15,
                                  cameraTargetPosition=[0, 0, 1.])
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    ## spawn draco in pybullet for visualization
    draco = pb.loadURDF(cwd + '/robot_model/draco/draco_modified.urdf',
                        INITIAL_POS,
                        INITIAL_QUAT,
                        useFixedBase=True)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        draco, INITIAL_POS, INITIAL_QUAT, False)

    nominal_sensor_data = pybullet_util.get_sensor_data(
        draco, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)

    joint_pos = copy.deepcopy(nominal_sensor_data['joint_pos'])

    robot_system = PinocchioRobotSystem(
        cwd + '/robot_model/draco/draco_modified.urdf',
        cwd + '/robot_model/draco', True, False)

    robot_system.update_system(nominal_sensor_data['base_joint_pos'],
                               nominal_sensor_data['base_joint_quat'],
                               nominal_sensor_data['base_joint_lin_vel'],
                               nominal_sensor_data['base_joint_ang_vel'],
                               joint_pos, nominal_sensor_data['joint_vel'],
                               True)

    nominal_inertia = robot_system._Ig[0:3, 0:3]

    data_saver = DataSaver('draco_cii.pkl')

    for r_haa in np.linspace(-np.pi / 4., np.pi / 4, num=30, endpoint=True):
        for r_hfe in np.linspace(-np.pi / 3, 0., num=30, endpoint=True):
            joint_pos['r_hip_aa'] = r_haa
            joint_pos['r_hip_fe'] = r_hfe

            robot_system.update_system(
                nominal_sensor_data['base_joint_pos'],
                nominal_sensor_data['base_joint_quat'],
                nominal_sensor_data['base_joint_lin_vel'],
                nominal_sensor_data['base_joint_ang_vel'], joint_pos,
                nominal_sensor_data['joint_vel'], True)

            inertia = robot_system._Ig[0:3, 0:3]

            ## compute CII
            CII = np.linalg.det(
                np.dot(inertia, np.linalg.inv(nominal_inertia)) - np.eye(3))
            # print("======CII======")
            # print(CII)

            ### for visualization
            # pb.resetJointState(draco, joint_id['r_hip_aa'], r_haa)
            # pb.resetJointState(draco, joint_id['r_hip_fe'], r_hfe)

            # time.sleep(0.1)

            ## data save
            data_saver.add('r_hip_aa', r_haa)
            data_saver.add('r_hip_fe', r_hfe)
            data_saver.add('cii', CII)
            data_saver.advance()

