import pybullet as pb
import time
import os

cwd = os.getcwd()
import sys

sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")

import time

import numpy as np
import ipdb

from config.draco.pybullet_simulation import Config
from util.python_utils import pybullet_util
from util.python_utils import util

import copy

import draco_interface_pybind


def set_initial_config(robot, joint_id):
    # Upperbody
    pb.resetJointState(robot, joint_id["l_shoulder_aa"], np.pi / 6, 0.)
    pb.resetJointState(robot, joint_id["l_elbow_fe"], -np.pi / 2, 0.)
    pb.resetJointState(robot, joint_id["r_shoulder_aa"], -np.pi / 6, 0.)
    pb.resetJointState(robot, joint_id["r_elbow_fe"], -np.pi / 2, 0.)

    # Lowerbody
    hip_yaw_angle = 5
    pb.resetJointState(robot, joint_id["l_hip_aa"], np.radians(hip_yaw_angle),
                       0.)
    pb.resetJointState(robot, joint_id["l_hip_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_knee_fe_jp"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_knee_fe_jd"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_ankle_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_ankle_ie"],
                       np.radians(-hip_yaw_angle), 0.)

    pb.resetJointState(robot, joint_id["r_hip_aa"], np.radians(-hip_yaw_angle),
                       0.)
    pb.resetJointState(robot, joint_id["r_hip_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_knee_fe_jp"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_knee_fe_jd"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_ankle_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_ankle_ie"],
                       np.radians(hip_yaw_angle), 0.)


pb.connect(pb.GUI)

pb.resetDebugVisualizerCamera(cameraDistance=1.5,
                              cameraYaw=90,
                              cameraPitch=0,
                              cameraTargetPosition=[0, 0, 0.8])
## sim physics setting
pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                             numSubSteps=Config.N_SUBSTEP)
pb.setGravity(0, 0, -9.81)

## robot spawn & initial kinematics and dynamics setting
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
draco_humanoid = pb.loadURDF(cwd + "/robot_model/draco/draco_modified.urdf",
                             Config.INITIAL_BASE_JOINT_POS,
                             Config.INITIAL_BASE_JOINT_QUAT,
                             useFixedBase=0)

ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=1)
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
    draco_humanoid, Config.INITIAL_BASE_JOINT_POS,
    Config.INITIAL_BASE_JOINT_QUAT, Config.PRINT_ROBOT_INFO)

#robot initial config setting
set_initial_config(draco_humanoid, joint_id)

#robot joint and link dynamics setting
pybullet_util.set_joint_friction(draco_humanoid, joint_id, 0)
pybullet_util.set_link_damping(draco_humanoid, link_id, 0., 0.)

## rolling contact joint constraint
c = pb.createConstraint(draco_humanoid,
                        link_id['l_knee_fe_lp'],
                        draco_humanoid,
                        link_id['l_knee_fe_ld'],
                        jointType=pb.JOINT_GEAR,
                        jointAxis=[0, 1, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0, 0, 0])
pb.changeConstraint(c, gearRatio=-1, maxForce=500, erp=10)

c = pb.createConstraint(draco_humanoid,
                        link_id['r_knee_fe_lp'],
                        draco_humanoid,
                        link_id['r_knee_fe_ld'],
                        jointType=pb.JOINT_GEAR,
                        jointAxis=[0, 1, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0, 0, 0])
pb.changeConstraint(c, gearRatio=-1, maxForce=500, erp=10)

#pnc interface, sensor_data, command class
pnc_draco_interface = draco_interface_pybind.DracoInterface()
pnc_draco_sensor_data = draco_interface_pybind.DracoSensorData()
pnc_draco_command = draco_interface_pybind.DracoCommand()

## Run Simulation
t = 0
dt = Config.CONTROLLER_DT
count = 0

while (True):

    ##get_sensor_data
    pybullet_sensor_data_dict = pybullet_util.get_sensor_data(
        draco_humanoid, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)

    pybullet_sensor_data_dict['imu_frame_iso'] = pybullet_util.get_link_iso(
        draco_humanoid, link_id['torso_imu'])
    pybullet_sensor_data_dict['imu_frame_vel'] = pybullet_util.get_link_vel(
        draco_humanoid, link_id['torso_imu'])
    lf_height = pybullet_util.get_link_iso(draco_humanoid,
                                           link_id['l_foot_contact'])[2, 3]
    rf_height = pybullet_util.get_link_iso(draco_humanoid,
                                           link_id['r_foot_contact'])[2, 3]
    pybullet_sensor_data_dict[
        'b_lf_contact'] = True if lf_height <= 0.05 else False
    pybullet_sensor_data_dict[
        'b_rf_contact'] = True if rf_height <= 0.05 else False

    ##copy pybullet sensor data to pnc sensor data
    pnc_draco_sensor_data.joint_positions_ = pybullet_sensor_data_dict[
        'joint_pos']
    pnc_draco_sensor_data.joint_velocities_ = pybullet_sensor_data_dict[
        'joint_vel']
    pnc_draco_sensor_data.imu_frame_isometry_ = pybullet_sensor_data_dict[
        'imu_frame_iso']
    pnc_draco_sensor_data.imu_frame_velocities_ = pybullet_sensor_data_dict[
        'imu_frame_vel']
    pnc_draco_sensor_data.b_lf_contact_ = pybullet_sensor_data_dict[
        'b_lf_contact']
    pnc_draco_sensor_data.b_rf_contact_ = pybullet_sensor_data_dict[
        'b_rf_contact']

    ##debugging
    pnc_draco_sensor_data.base_com_pos_ = pybullet_sensor_data_dict[
        'base_com_pos']
    pnc_draco_sensor_data.base_com_quat_ = pybullet_sensor_data_dict[
        'base_com_quat']
    pnc_draco_sensor_data.base_com_lin_vel_ = pybullet_sensor_data_dict[
        'base_com_lin_vel']
    pnc_draco_sensor_data.base_com_ang_vel_ = pybullet_sensor_data_dict[
        'base_com_ang_vel']

    pnc_draco_sensor_data.base_joint_pos_ = pybullet_sensor_data_dict[
        'base_joint_pos']
    pnc_draco_sensor_data.base_joint_quat_ = pybullet_sensor_data_dict[
        'base_joint_quat']
    pnc_draco_sensor_data.base_joint_lin_vel_ = pybullet_sensor_data_dict[
        'base_joint_lin_vel']
    pnc_draco_sensor_data.base_joint_ang_vel_ = pybullet_sensor_data_dict[
        'base_joint_ang_vel']

    ##Debugging purpose
    # print("==================")
    # print("pybullet robot")
    # print("==================")
    # print("base com pos")
    # print(pybullet_sensor_data_dict['base_com_pos'])
    # print("base com ori")
    # print(util.quat_to_rot(pybullet_sensor_data_dict['base_com_quat']))
    # print("base joint pos")
    # print(pybullet_sensor_data_dict['base_joint_pos'])
    # print("base joint ori")
    # print(util.quat_to_rot(pybullet_sensor_data_dict['base_joint_quat']))
    # print("base_com_lin_vel")
    # print(pybullet_sensor_data_dict['base_com_lin_vel'])
    # print("base_com_ang_vel")
    # print(pybullet_sensor_data_dict['base_com_ang_vel'])
    # print("base_joint_lin_vel")
    # print(pybullet_sensor_data_dict['base_joint_lin_vel'])
    # print("base_joint_ang_vel")
    # print(pybullet_sensor_data_dict['base_joint_ang_vel'])
    # print("lf height")
    # print(lf_height)

    # lf_pos = pybullet_util.get_link_iso(draco_humanoid,
    # link_id['l_foot_contact'])[:3, 3]
    # print("lf pos")
    # print(lf_pos)

    ##compute control command
    pnc_draco_interface.GetCommand(pnc_draco_sensor_data, pnc_draco_command)

    ##copy command
    pybullet_joint_positions_cmd = copy.deepcopy(
        pnc_draco_command.joint_positions_cmd_)
    pybullet_joint_velocities_cmd = copy.deepcopy(
        pnc_draco_command.joint_velocities_cmd_)
    pybullet_joint_torques_cmd = copy.deepcopy(
        pnc_draco_command.joint_torques_cmd_)

    # if count == 9 or count == 10:
    # print('count', count)
    # print('jpos sensor data', pnc_draco_sensor_data.joint_positions_)
    # print('========================================================')
    # print('jpos command data', pnc_draco_command.joint_positions_cmd_)
    # __import__('ipdb').set_trace()

    ##delete passive joint command
    del pybullet_joint_positions_cmd['l_knee_fe_jp']
    del pybullet_joint_velocities_cmd['l_knee_fe_jp']
    del pybullet_joint_torques_cmd['l_knee_fe_jp']
    del pybullet_joint_positions_cmd['r_knee_fe_jp']
    del pybullet_joint_velocities_cmd['r_knee_fe_jp']
    del pybullet_joint_torques_cmd['r_knee_fe_jp']

    ##apply motor torque
    pybullet_util.set_motor_trq(draco_humanoid, joint_id,
                                pybullet_joint_torques_cmd)
    print("trq command value")
    print(pybullet_joint_torques_cmd)
    # pybullet_util.set_motor_pos(draco_humanoid, joint_id,
    # pybullet_joint_positions_cmd)
    #step simulation
    pb.stepSimulation()
    # time.sleep(dt)

    t += dt
    count += 1
