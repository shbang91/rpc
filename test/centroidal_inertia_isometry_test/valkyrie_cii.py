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
    ## spawn valkyrie in pybullet for visualization
    valkyrie = pb.loadURDF(cwd + '/robot_model/valkyrie/valkyrie.urdf',
                           INITIAL_POS,
                           INITIAL_QUAT,
                           useFixedBase=True)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        valkyrie, INITIAL_POS, INITIAL_QUAT, False)

    nominal_sensor_data = pybullet_util.get_sensor_data(
        valkyrie, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)

    joint_pos = copy.deepcopy(nominal_sensor_data['joint_pos'])
    # joint_pos['rightShoulderRoll'] = np.pi / 2
    # joint_pos['leftShoulderRoll'] = -np.pi / 2

    robot_system = PinocchioRobotSystem(
        cwd + '/robot_model/valkyrie/valkyrie.urdf',
        cwd + '/robot_model/valkyrie', True, False)

    robot_system.update_system(nominal_sensor_data['base_joint_pos'],
                               nominal_sensor_data['base_joint_quat'],
                               nominal_sensor_data['base_joint_lin_vel'],
                               nominal_sensor_data['base_joint_ang_vel'],
                               joint_pos, nominal_sensor_data['joint_vel'],
                               True)

    nominal_inertia = robot_system._Ig[0:3, 0:3]

    data_saver = DataSaver('valkyrie_cii.pkl')

    # for haa in np.linspace(0., 30. * np.pi / 180., num=30, endpoint=True):
        # for hfe in np.linspace(-np.pi / 6., 0., num=30, endpoint=True):
    for haa in np.linspace(0. * np.pi/180, 50. * np.pi / 180., num=50, endpoint=True):
        for hfe in np.linspace(-np.pi / 3., 0., num=50, endpoint=True):
            joint_pos['rightHipRoll'] = -haa
            joint_pos['rightHipPitch'] = hfe
            joint_pos['rightKneePitch'] = -2 * hfe
            joint_pos['leftHipRoll'] = haa
            joint_pos['leftHipPitch'] = hfe
            joint_pos['leftKneePitch'] = -2 * hfe

            robot_system.update_system(
                nominal_sensor_data['base_joint_pos'],
                nominal_sensor_data['base_joint_quat'],
                nominal_sensor_data['base_joint_lin_vel'],
                nominal_sensor_data['base_joint_ang_vel'], joint_pos,
                nominal_sensor_data['joint_vel'], True)

            inertia = robot_system._Ig[0:3, 0:3]

            # compute CII
            CII = np.linalg.det(
                np.dot(np.linalg.inv(inertia), nominal_inertia) - np.eye(3))

            # for visualization
            # pb.resetJointState(valkyrie, joint_id['rightShoulderRoll'],
                               # np.pi / 2)
            # pb.resetJointState(valkyrie, joint_id['leftShoulderRoll'],
                               # -np.pi / 2)
            pb.resetJointState(valkyrie, joint_id['rightHipRoll'], -haa)
            pb.resetJointState(valkyrie, joint_id['rightHipPitch'], hfe)
            pb.resetJointState(valkyrie, joint_id['rightKneePitch'], -2 * hfe)
            pb.resetJointState(valkyrie, joint_id['leftHipRoll'], haa)
            pb.resetJointState(valkyrie, joint_id['leftHipPitch'], hfe)
            pb.resetJointState(valkyrie, joint_id['leftKneePitch'], -2 * hfe)

            time.sleep(0.001)

            # data save
            data_saver.add('cii', CII)
            data_saver.advance()
