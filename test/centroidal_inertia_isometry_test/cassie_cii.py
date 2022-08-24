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

VIDEO_RECORD = True

if __name__ == '__main__':
    #pybullet env
    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=2.0,
                                  cameraYaw=120,
                                  cameraPitch=-15,
                                  cameraTargetPosition=[0, 0, 1.])
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    ## video recording
    if VIDEO_RECORD:
        if not os.path.exists('video'):
            os.makedirs('video')
        for f in os.listdir('video'):
            if f == "cassie_cii.mp4":
                os.remove('video/' + f)
        pb.startStateLogging(pb.STATE_LOGGING_VIDEO_MP4, "video/cassie_cii.mp4")


    ## spawn cassie in pybullet for visualization
    cassie = pb.loadURDF(cwd + '/robot_model/cassie/urdf/cassie.urdf',
                        INITIAL_POS,
                        INITIAL_QUAT,
                        useFixedBase=True)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        cassie, INITIAL_POS, INITIAL_QUAT, False)

    nominal_sensor_data = pybullet_util.get_sensor_data(
        cassie, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)

    joint_pos = copy.deepcopy(nominal_sensor_data['joint_pos'])
    joint_pos['toe_joint_left'] = -np.pi/2
    joint_pos['toe_joint_right'] = -np.pi/2


    robot_system = PinocchioRobotSystem(cwd + '/robot_model/cassie/urdf/cassie.urdf',
                                        cwd + '/robot_model/cassie', True,
                                        False)

    robot_system.update_system(nominal_sensor_data['base_joint_pos'],
                               nominal_sensor_data['base_joint_quat'],
                               nominal_sensor_data['base_joint_lin_vel'],
                               nominal_sensor_data['base_joint_ang_vel'],
                               joint_pos, nominal_sensor_data['joint_vel'],
                               True)

    nominal_inertia = robot_system.Ig[0:3, 0:3]

    data_saver = DataSaver('cassie_cii.pkl')

    for r_haa in np.linspace(-np.pi / 4., np.pi / 4, num=30, endpoint=True):
        for r_hfe in np.linspace(-np.pi / 3, 0., num=30, endpoint=True):
            joint_pos['hip_abduction_right'] = r_haa
            joint_pos['hip_flexion_right'] = r_hfe
            joint_pos['ankle_joint_right'] = -2.5*r_hfe

            robot_system.update_system(
                nominal_sensor_data['base_joint_pos'],
                nominal_sensor_data['base_joint_quat'],
                nominal_sensor_data['base_joint_lin_vel'],
                nominal_sensor_data['base_joint_ang_vel'], joint_pos,
                nominal_sensor_data['joint_vel'], True)

            inertia = robot_system.Ig[0:3, 0:3]

            # compute CII
            CII = np.linalg.det(
                np.dot(inertia, np.linalg.inv(nominal_inertia)) - np.eye(3))

            # for visualization
            pb.resetJointState(cassie, joint_id['toe_joint_left'], -np.pi/2)
            pb.resetJointState(cassie, joint_id['toe_joint_right'], -np.pi/2)
            pb.resetJointState(cassie, joint_id['hip_abduction_right'], r_haa)
            pb.resetJointState(cassie, joint_id['hip_flexion_right'], r_hfe)
            pb.resetJointState(cassie, joint_id['ankle_joint_right'], -2.5*r_hfe)

            time.sleep(0.01)

            # data save
            data_saver.add('cii', CII)
            data_saver.advance()
