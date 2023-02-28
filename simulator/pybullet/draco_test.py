import pybullet as pb
import time
import os

cwd = os.getcwd()
import sys

#cwd = "~/Projects/draco3/rpc_mingyo"
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  #include pybind module

import time
import datetime

import numpy as np

from config.draco.pybullet_simulation import *
from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

import copy

import draco_interface_py
import cv2
# from pytictoc import TicToc

import argparse
import h5py

parser = argparse.ArgumentParser()
parser.add_argument("--mode", type=str, default='gui', help="")
parser.add_argument("--path", type=str, default='./', help="")

args = parser.parse_args()
save_path = args.path
render_mode = args.mode


def get_sensor_data_from_pybullet(robot):

    #follow pinocchio robotsystem urdf reading convention
    joint_pos, joint_vel = np.zeros(27), np.zeros(27)

    imu_frame_quat = np.array(
        pb.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[1])
    #LF
    joint_pos[0] = pb.getJointState(robot, DracoJointIdx.l_hip_ie)[0]
    joint_pos[1] = pb.getJointState(robot, DracoJointIdx.l_hip_aa)[0]
    joint_pos[2] = pb.getJointState(robot, DracoJointIdx.l_hip_fe)[0]
    joint_pos[3] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[0]
    joint_pos[4] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[0]
    joint_pos[5] = pb.getJointState(robot, DracoJointIdx.l_ankle_fe)[0]
    joint_pos[6] = pb.getJointState(robot, DracoJointIdx.l_ankle_ie)[0]
    #LH
    joint_pos[7] = pb.getJointState(robot, DracoJointIdx.l_shoulder_fe)[0]
    joint_pos[8] = pb.getJointState(robot, DracoJointIdx.l_shoulder_aa)[0]
    joint_pos[9] = pb.getJointState(robot, DracoJointIdx.l_shoulder_ie)[0]
    joint_pos[10] = pb.getJointState(robot, DracoJointIdx.l_elbow_fe)[0]
    joint_pos[11] = pb.getJointState(robot, DracoJointIdx.l_wrist_ps)[0]
    joint_pos[12] = pb.getJointState(robot, DracoJointIdx.l_wrist_pitch)[0]
    #neck
    joint_pos[13] = pb.getJointState(robot, DracoJointIdx.neck_pitch)[0]
    #RF
    joint_pos[14] = pb.getJointState(robot, DracoJointIdx.r_hip_ie)[0]
    joint_pos[15] = pb.getJointState(robot, DracoJointIdx.r_hip_aa)[0]
    joint_pos[16] = pb.getJointState(robot, DracoJointIdx.r_hip_fe)[0]
    joint_pos[17] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[0]
    joint_pos[18] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[0]
    joint_pos[19] = pb.getJointState(robot, DracoJointIdx.r_ankle_fe)[0]
    joint_pos[20] = pb.getJointState(robot, DracoJointIdx.r_ankle_ie)[0]
    #RH
    joint_pos[21] = pb.getJointState(robot, DracoJointIdx.r_shoulder_fe)[0]
    joint_pos[22] = pb.getJointState(robot, DracoJointIdx.r_shoulder_aa)[0]
    joint_pos[23] = pb.getJointState(robot, DracoJointIdx.r_shoulder_ie)[0]
    joint_pos[24] = pb.getJointState(robot, DracoJointIdx.r_elbow_fe)[0]
    joint_pos[25] = pb.getJointState(robot, DracoJointIdx.r_wrist_ps)[0]
    joint_pos[26] = pb.getJointState(robot, DracoJointIdx.r_wrist_pitch)[0]

    imu_ang_vel = np.array(
        pb.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[7])
    #LF
    joint_vel[0] = pb.getJointState(robot, DracoJointIdx.l_hip_ie)[1]
    joint_vel[1] = pb.getJointState(robot, DracoJointIdx.l_hip_aa)[1]
    joint_vel[2] = pb.getJointState(robot, DracoJointIdx.l_hip_fe)[1]
    joint_vel[3] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[1]
    joint_vel[4] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[1]
    joint_vel[5] = pb.getJointState(robot, DracoJointIdx.l_ankle_fe)[1]
    joint_vel[6] = pb.getJointState(robot, DracoJointIdx.l_ankle_ie)[1]
    #LH
    joint_vel[7] = pb.getJointState(robot, DracoJointIdx.l_shoulder_fe)[1]
    joint_vel[8] = pb.getJointState(robot, DracoJointIdx.l_shoulder_aa)[1]
    joint_vel[9] = pb.getJointState(robot, DracoJointIdx.l_shoulder_ie)[1]
    joint_vel[10] = pb.getJointState(robot, DracoJointIdx.l_elbow_fe)[1]
    joint_vel[11] = pb.getJointState(robot, DracoJointIdx.l_wrist_ps)[1]
    joint_vel[12] = pb.getJointState(robot, DracoJointIdx.l_wrist_pitch)[1]
    #neck
    joint_vel[13] = pb.getJointState(robot, DracoJointIdx.neck_pitch)[1]
    #RF
    joint_vel[14] = pb.getJointState(robot, DracoJointIdx.r_hip_ie)[1]
    joint_vel[15] = pb.getJointState(robot, DracoJointIdx.r_hip_aa)[1]
    joint_vel[16] = pb.getJointState(robot, DracoJointIdx.r_hip_fe)[1]
    joint_vel[17] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[1]
    joint_vel[18] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[1]
    joint_vel[19] = pb.getJointState(robot, DracoJointIdx.r_ankle_fe)[1]
    joint_vel[20] = pb.getJointState(robot, DracoJointIdx.r_ankle_ie)[1]
    #RH
    joint_vel[21] = pb.getJointState(robot, DracoJointIdx.r_shoulder_fe)[1]
    joint_vel[22] = pb.getJointState(robot, DracoJointIdx.r_shoulder_aa)[1]
    joint_vel[23] = pb.getJointState(robot, DracoJointIdx.r_shoulder_ie)[1]
    joint_vel[24] = pb.getJointState(robot, DracoJointIdx.r_elbow_fe)[1]
    joint_vel[25] = pb.getJointState(robot, DracoJointIdx.r_wrist_ps)[1]
    joint_vel[26] = pb.getJointState(robot, DracoJointIdx.r_wrist_pitch)[1]

    b_lf_contact = True if pb.getLinkState(robot, DracoLinkIdx.l_foot_contact,
                                           1, 1)[0][2] <= 0.05 else False
    b_rf_contact = True if pb.getLinkState(robot, DracoLinkIdx.r_foot_contact,
                                           1, 1)[0][2] <= 0.05 else False
    return imu_frame_quat, imu_ang_vel, joint_pos, joint_vel, b_lf_contact, b_rf_contact


#TODO:try to modify with setjointmotorcontrol "array" API
def apply_control_input_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL

    #LF
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_ie,
                             controlMode=mode,
                             force=command[0])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_aa,
                             controlMode=mode,
                             force=command[1])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_fe,
                             controlMode=mode,
                             force=command[2])
    # pb.setJointMotorControl2(robot, DracoJointIdx.l_knee_fe_jp, controlMode=mode, force=command[3])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_knee_fe_jd,
                             controlMode=mode,
                             force=command[4])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_ankle_fe,
                             controlMode=mode,
                             force=command[5])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_ankle_ie,
                             controlMode=mode,
                             force=command[6])

    #LH
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_fe,
                             controlMode=mode,
                             force=command[7])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_aa,
                             controlMode=mode,
                             force=command[8])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_ie,
                             controlMode=mode,
                             force=command[9])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_elbow_fe,
                             controlMode=mode,
                             force=command[10])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_wrist_ps,
                             controlMode=mode,
                             force=command[11])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_wrist_pitch,
                             controlMode=mode,
                             force=command[12])

    #neck
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.neck_pitch,
                             controlMode=mode,
                             force=command[13])

    #RF
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_ie,
                             controlMode=mode,
                             force=command[14])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_aa,
                             controlMode=mode,
                             force=command[15])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_fe,
                             controlMode=mode,
                             force=command[16])
    # pb.setJointMotorControl2(robot, DracoJointIdx.r_knee_fe_jd, controlMode=mode, force=command[17])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_knee_fe_jd,
                             controlMode=mode,
                             force=command[18])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_ankle_fe,
                             controlMode=mode,
                             force=command[19])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_ankle_ie,
                             controlMode=mode,
                             force=command[20])

    #RH
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_fe,
                             controlMode=mode,
                             force=command[21])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_aa,
                             controlMode=mode,
                             force=command[22])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_ie,
                             controlMode=mode,
                             force=command[23])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_elbow_fe,
                             controlMode=mode,
                             force=command[24])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_wrist_ps,
                             controlMode=mode,
                             force=command[25])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_wrist_pitch,
                             controlMode=mode,
                             force=command[26])


def set_init_config_pybullet_robot(robot):
    # Upperbody
    pb.resetJointState(robot, DracoJointIdx.l_shoulder_aa, np.pi / 6, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_elbow_fe, -np.pi / 2, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_shoulder_aa, -np.pi / 6, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_elbow_fe, -np.pi / 2, 0.)

    # Lowerbody
    hip_yaw_angle = 0
    pb.resetJointState(robot, DracoJointIdx.l_hip_aa,
                       np.radians(hip_yaw_angle), 0.)
    pb.resetJointState(robot, DracoJointIdx.l_hip_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_knee_fe_jp, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_knee_fe_jd, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_ankle_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_ankle_ie,
                       np.radians(-hip_yaw_angle), 0.)

    pb.resetJointState(robot, DracoJointIdx.r_hip_aa,
                       np.radians(-hip_yaw_angle), 0.)
    pb.resetJointState(robot, DracoJointIdx.r_hip_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_knee_fe_jp, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_knee_fe_jd, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_ankle_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_ankle_ie,
                       np.radians(hip_yaw_angle), 0.)


if render_mode == 'gui':
    pb.connect(pb.GUI)
else:
    pb.connect(pb.DIRECT)

pb.resetDebugVisualizerCamera(cameraDistance=1.5,
                              cameraYaw=120,
                              cameraPitch=-30,
                              cameraTargetPosition=[0, 0, 0.5])
## sim physics setting
pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                             numSubSteps=Config.N_SUBSTEP)
pb.setGravity(0, 0, -9.81)

## robot spawn & initial kinematics and dynamics setting
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
# draco_humanoid = pb.loadURDF(cwd + "/robot_model/draco/draco_modified.urdf",
# Config.INITIAL_BASE_JOINT_POS,
# Config.INITIAL_BASE_JOINT_QUAT,
# useFixedBase=0)
draco_humanoid = pb.loadURDF(cwd + "/robot_model/draco/draco3_big_feet.urdf",
                             Config.INITIAL_BASE_JOINT_POS,
                             Config.INITIAL_BASE_JOINT_QUAT,
                             useFixedBase=0)

ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=1)
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

#TODO:modify this function without dictionary container
n_q, n_v, n_a, joint_id_dict, link_id_dict, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
    draco_humanoid, Config.INITIAL_BASE_JOINT_POS,
    Config.INITIAL_BASE_JOINT_QUAT, Config.PRINT_ROBOT_INFO)

#robot initial config setting
set_init_config_pybullet_robot(draco_humanoid)

#robot joint and link dynamics setting
#TODO:modify this function without dictionary container
pybullet_util.set_joint_friction(draco_humanoid, joint_id_dict, 0)
pybullet_util.set_link_damping(draco_humanoid, link_id_dict, 0., 0.)

## rolling contact joint constraint
c1 = pb.createConstraint(draco_humanoid,
                         DracoLinkIdx.l_knee_fe_lp,
                         draco_humanoid,
                         DracoLinkIdx.l_knee_fe_ld,
                         jointType=pb.JOINT_GEAR,
                         jointAxis=[0, 1, 0],
                         parentFramePosition=[0, 0, 0],
                         childFramePosition=[0, 0, 0])
pb.changeConstraint(c1, gearRatio=-1, maxForce=500, erp=10)

c2 = pb.createConstraint(draco_humanoid,
                         DracoLinkIdx.r_knee_fe_lp,
                         draco_humanoid,
                         DracoLinkIdx.r_knee_fe_ld,
                         jointType=pb.JOINT_GEAR,
                         jointAxis=[0, 1, 0],
                         parentFramePosition=[0, 0, 0],
                         childFramePosition=[0, 0, 0])
pb.changeConstraint(c2, gearRatio=-1, maxForce=500, erp=10)

#pnc interface, sensor_data, command class
rpc_draco_interface = draco_interface_py.DracoInterface()
rpc_draco_sensor_data = draco_interface_py.DracoSensorData()
rpc_draco_command = draco_interface_py.DracoCommand()

#TODO
#active jointidx list in sequence
active_jointidx_list = []  #for setjointmotorcontrolarray

#default robot kinematics information
base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(draco_humanoid)
rot_world_basecom = util.quat_to_rot(np.array(base_com_quat))
rot_world_basejoint = util.quat_to_rot(np.array(
    Config.INITIAL_BASE_JOINT_QUAT))

pos_basejoint_to_basecom = np.dot(
    rot_world_basejoint.transpose(),
    base_com_pos - np.array(Config.INITIAL_BASE_JOINT_POS))
rot_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                  rot_world_basecom)

recording_name = datetime.datetime.now().strftime("%m%d_%H%M%S")
joint_state = {
    'time': [],
    'act_pos': [],
    'act_vel': [],
    'des_pos': [],
    'des_vel': [],
}

if render_mode == 'gui':
    video_format = cv2.VideoWriter_fourcc(*'mp4v')
    render_width = 480
    render_height = 360
    recorder = cv2.VideoWriter(
        os.path.join(save_path, '{}.mp4'.format(recording_name)), video_format,
        30, (render_width, render_height))

    roll = 0.0
    pitch = -30.0  # * np.pi/180.
    yaw = 60.0  # * np.pi/180.
    record_time = 0.0

# Run Simulation
t = 0
dt = Config.CONTROLLER_DT
count = 0

# timer = TicToc()
# compuation_cal_list = []

while (True):

    ###############################################################################
    #Debugging Purpose
    ##############################################################################
    ##debugging state estimator by calculating groundtruth basejoint states
    base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(
        draco_humanoid)

    rot_world_basecom = util.quat_to_rot(base_com_quat)
    rot_world_basejoint = np.dot(rot_world_basecom,
                                 rot_basejoint_to_basecom.transpose())
    base_joint_pos = base_com_pos - np.dot(rot_world_basejoint,
                                           pos_basejoint_to_basecom)
    base_joint_quat = util.rot_to_quat(rot_world_basejoint)

    base_com_lin_vel, base_com_ang_vel = pb.getBaseVelocity(draco_humanoid)
    trans_joint_com = liegroup.RpToTrans(rot_basejoint_to_basecom,
                                         pos_basejoint_to_basecom)
    adjoint_joint_com = liegroup.Adjoint(trans_joint_com)
    twist_basecom_in_world = np.zeros(6)
    twist_basecom_in_world[0:3] = base_com_ang_vel
    twist_basecom_in_world[3:6] = base_com_lin_vel
    augrot_basecom_world = np.zeros((6, 6))
    augrot_basecom_world[0:3, 0:3] = rot_world_basecom.transpose()
    augrot_basecom_world[3:6, 3:6] = rot_world_basecom.transpose()
    twist_basecom_in_basecom = np.dot(augrot_basecom_world,
                                      twist_basecom_in_world)
    twist_basejoint_in_basejoint = np.dot(adjoint_joint_com,
                                          twist_basecom_in_basecom)
    augrot_world_basejoint = np.zeros((6, 6))
    augrot_world_basejoint[0:3, 0:3] = rot_world_basejoint
    augrot_world_basejoint[3:6, 3:6] = rot_world_basejoint
    twist_basejoint_in_world = np.dot(augrot_world_basejoint,
                                      twist_basejoint_in_basejoint)
    base_joint_ang_vel = twist_basejoint_in_world[0:3]
    base_joint_lin_vel = twist_basejoint_in_world[3:6]

    #pass debugged data to rpc interface
    rpc_draco_sensor_data.base_joint_pos_ = base_joint_pos
    rpc_draco_sensor_data.base_joint_quat_ = base_joint_quat
    rpc_draco_sensor_data.base_joint_lin_vel_ = base_joint_lin_vel
    rpc_draco_sensor_data.base_joint_ang_vel_ = base_joint_ang_vel
    ##############################################################################
    ##############################################################################

    # Get Keyboard Event
    keys = pb.getKeyboardEvents()
    if pybullet_util.is_key_triggered(keys, '1'):
        rpc_draco_interface.interrupt_.PressOne()
    if pybullet_util.is_key_triggered(keys, '2'):
        rpc_draco_interface.interrupt_.PressTwo()
    if pybullet_util.is_key_triggered(keys, '4'):
        rpc_draco_interface.interrupt_.PressFour()
    if pybullet_util.is_key_triggered(keys, '5'):
        rpc_draco_interface.interrupt_.PressFive()
    if pybullet_util.is_key_triggered(keys, '6'):
        rpc_draco_interface.interrupt_.PressSix()
    if pybullet_util.is_key_triggered(keys, '7'):
        rpc_draco_interface.interrupt_.PressSeven()
    if pybullet_util.is_key_triggered(keys, '8'):
        rpc_draco_interface.interrupt_.PressEight()
    if pybullet_util.is_key_triggered(keys, '9'):
        rpc_draco_interface.interrupt_.PressNine()
    if pybullet_util.is_key_triggered(keys, 's'):
        np.savetxt('computation_time.txt',
                   np.array([compuation_cal_list]),
                   delimiter=',')

    # if t> 7:
    # rpc_draco_interface.interrupt_.PressEight()

    #get sensor data
    imu_frame_quat, imu_ang_vel, joint_pos, joint_vel, b_lf_contact, b_rf_contact = get_sensor_data_from_pybullet(
        draco_humanoid)

    #copy sensor data to rpc sensor data class
    rpc_draco_sensor_data.imu_frame_quat_ = imu_frame_quat
    rpc_draco_sensor_data.imu_ang_vel_ = imu_ang_vel
    rpc_draco_sensor_data.joint_pos_ = joint_pos
    rpc_draco_sensor_data.joint_vel_ = joint_vel
    rpc_draco_sensor_data.b_lf_contact_ = b_lf_contact
    rpc_draco_sensor_data.b_rf_contact_ = b_rf_contact

    ##Debugging

    ##compute control command
    # timer.tic()
    rpc_draco_interface.GetCommand(rpc_draco_sensor_data, rpc_draco_command)

    # comp_time = timer.tocvalue()
    # compuation_cal_list.append(comp_time)

    #copy command data from rpc command class
    rpc_trq_command = rpc_draco_command.joint_trq_cmd_
    rpc_joint_pos_command = rpc_draco_command.joint_pos_cmd_
    rpc_joint_vel_command = rpc_draco_command.joint_vel_cmd_

    #apply command to pybullet robot
    apply_control_input_to_pybullet(draco_humanoid, rpc_trq_command)

    # lfoot_pos = pybullet_util.get_link_iso(draco_humanoid,
    # DracoLinkIdx.l_foot_contact)[0:3, 3]
    # rfoot_pos = pybullet_util.get_link_iso(draco_humanoid,
    # DracoLinkIdx.r_foot_contact)[0:3, 3]
    # print("------------------------------------")
    # print(rfoot_pos[1] - lfoot_pos[1])

    # print("trq command printout")
    # print(rpc_trq_command)
    # print("jpos command printout")
    # print(rpc_joint_pos_command)
    # print("jpos command printout")
    # print(rpc_joint_vel_command)

    #step simulation
    pb.stepSimulation()

    joint_state['time'].append(t)
    joint_state['act_pos'].append(joint_pos)
    joint_state['act_vel'].append(joint_vel)
    joint_state['des_pos'].append(rpc_joint_pos_command)
    joint_state['des_vel'].append(rpc_joint_vel_command)

    if render_mode == 'gui' and t > record_time + 1.0 / 30:
        position = np.array([0.0, 0.0, 0.75])
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        view_point, _ = pb.multiplyTransforms(position, orientation,
                                              [0.045, 0.0, 0.0], [0, 0, 0, 1])
        view_rpy = pb.getEulerFromQuaternion(orientation)
        view_matrix = pb.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=view_point,
            distance=2.0,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            upAxisIndex=2)
        proj_matrix = pb.computeProjectionMatrixFOV(
            fov=60,
            aspect=float(render_width) / render_height,
            nearVal=0.1,
            farVal=100)
        (_, _, rgb, depth,
         _) = pb.getCameraImage(width=render_width,
                                height=render_height,
                                renderer=pb.ER_BULLET_HARDWARE_OPENGL,
                                viewMatrix=view_matrix,
                                shadow=0,
                                projectionMatrix=proj_matrix)
        img = np.array(rgb[:, :, [2, 1, 0]])
        recorder.write(img)
        record_time += 1.0 / 30

    t += dt
    count += 1

    if t > 20.0:
        break

recorder.release()
dataset = h5py.File(os.path.join(save_path, '{}.hdf5'.format(recording_name)),
                    'w')
dataset.create_dataset('act_pos',
                       data=np.array(joint_state['act_pos']),
                       compression="gzip",
                       chunks=True,
                       dtype='f')
dataset.create_dataset('act_vel',
                       data=np.array(joint_state['act_vel']),
                       compression="gzip",
                       chunks=True,
                       dtype='f')
dataset.create_dataset('des_pos',
                       data=np.array(joint_state['des_pos']),
                       compression="gzip",
                       chunks=True,
                       dtype='f')
dataset.create_dataset('des_vel',
                       data=np.array(joint_state['des_vel']),
                       compression="gzip",
                       chunks=True,
                       dtype='f')
dataset.create_dataset('time',
                       data=np.array(joint_state['time']),
                       compression="gzip",
                       chunks=True,
                       dtype='f')
dataset.close()
