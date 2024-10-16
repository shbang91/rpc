import pybullet as pb
import os
import sys
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

from config.draco.sim.pybullet.ihwbc.pybullet_params import *
from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

import signal
import shutil
import cv2
import draco_interface_py

if Config.MEASURE_COMPUTATION_TIME:
    from pytictoc import TicToc

# if Config.USE_FOXGLOVE:
#     import UI.foxglove.control_widgets as foxglove_ctrl
#     import asyncio
#     import threading
#
#     # load parameters that can be controlled / changed
#     param_store = foxglove_ctrl.load_params_store()
#     step_listener = foxglove_ctrl.Listener(param_store)

# start the foxglove server on separate thread
# x = threading.Thread(target=asyncio.run, args=([foxglove_ctrl.run(step_listener)]))
# x.start()

# Simulated noise characteristics
imu_dvel_bias = np.array([0.0, 0.0, 0.0])
l_contact_volt_noise = 0.001
r_contact_volt_noise = 0.001
imu_ang_vel_noise_std_dev = 0.0  # based on real IMU: 0.0052


def get_sensor_data_from_pybullet(robot):
    # follow pinocchio robotsystem urdf reading convention
    joint_pos, joint_vel = np.zeros(27), np.zeros(27)

    imu_frame_quat = np.array(
        pb.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[1])
    # LF
    joint_pos[0] = pb.getJointState(robot, DracoJointIdx.l_hip_ie)[0]
    joint_pos[1] = pb.getJointState(robot, DracoJointIdx.l_hip_aa)[0]
    joint_pos[2] = pb.getJointState(robot, DracoJointIdx.l_hip_fe)[0]
    joint_pos[3] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[0]
    joint_pos[4] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[0]
    joint_pos[5] = pb.getJointState(robot, DracoJointIdx.l_ankle_fe)[0]
    joint_pos[6] = pb.getJointState(robot, DracoJointIdx.l_ankle_ie)[0]
    # LH
    joint_pos[7] = pb.getJointState(robot, DracoJointIdx.l_shoulder_fe)[0]
    joint_pos[8] = pb.getJointState(robot, DracoJointIdx.l_shoulder_aa)[0]
    joint_pos[9] = pb.getJointState(robot, DracoJointIdx.l_shoulder_ie)[0]
    joint_pos[10] = pb.getJointState(robot, DracoJointIdx.l_elbow_fe)[0]
    joint_pos[11] = pb.getJointState(robot, DracoJointIdx.l_wrist_ps)[0]
    joint_pos[12] = pb.getJointState(robot, DracoJointIdx.l_wrist_pitch)[0]
    # neck
    joint_pos[13] = pb.getJointState(robot, DracoJointIdx.neck_pitch)[0]
    # RF
    joint_pos[14] = pb.getJointState(robot, DracoJointIdx.r_hip_ie)[0]
    joint_pos[15] = pb.getJointState(robot, DracoJointIdx.r_hip_aa)[0]
    joint_pos[16] = pb.getJointState(robot, DracoJointIdx.r_hip_fe)[0]
    joint_pos[17] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[0]
    joint_pos[18] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[0]
    joint_pos[19] = pb.getJointState(robot, DracoJointIdx.r_ankle_fe)[0]
    joint_pos[20] = pb.getJointState(robot, DracoJointIdx.r_ankle_ie)[0]
    # RH
    joint_pos[21] = pb.getJointState(robot, DracoJointIdx.r_shoulder_fe)[0]
    joint_pos[22] = pb.getJointState(robot, DracoJointIdx.r_shoulder_aa)[0]
    joint_pos[23] = pb.getJointState(robot, DracoJointIdx.r_shoulder_ie)[0]
    joint_pos[24] = pb.getJointState(robot, DracoJointIdx.r_elbow_fe)[0]
    joint_pos[25] = pb.getJointState(robot, DracoJointIdx.r_wrist_ps)[0]
    joint_pos[26] = pb.getJointState(robot, DracoJointIdx.r_wrist_pitch)[0]

    imu_ang_vel = np.array(
        pb.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[7])

    imu_dvel = pybullet_util.simulate_dVel_data(robot, DracoLinkIdx.torso_imu,
                                                previous_torso_velocity)

    # LF
    joint_vel[0] = pb.getJointState(robot, DracoJointIdx.l_hip_ie)[1]
    joint_vel[1] = pb.getJointState(robot, DracoJointIdx.l_hip_aa)[1]
    joint_vel[2] = pb.getJointState(robot, DracoJointIdx.l_hip_fe)[1]
    joint_vel[3] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[1]
    joint_vel[4] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[1]
    joint_vel[5] = pb.getJointState(robot, DracoJointIdx.l_ankle_fe)[1]
    joint_vel[6] = pb.getJointState(robot, DracoJointIdx.l_ankle_ie)[1]
    # LH
    joint_vel[7] = pb.getJointState(robot, DracoJointIdx.l_shoulder_fe)[1]
    joint_vel[8] = pb.getJointState(robot, DracoJointIdx.l_shoulder_aa)[1]
    joint_vel[9] = pb.getJointState(robot, DracoJointIdx.l_shoulder_ie)[1]
    joint_vel[10] = pb.getJointState(robot, DracoJointIdx.l_elbow_fe)[1]
    joint_vel[11] = pb.getJointState(robot, DracoJointIdx.l_wrist_ps)[1]
    joint_vel[12] = pb.getJointState(robot, DracoJointIdx.l_wrist_pitch)[1]
    # neck
    joint_vel[13] = pb.getJointState(robot, DracoJointIdx.neck_pitch)[1]
    # RF
    joint_vel[14] = pb.getJointState(robot, DracoJointIdx.r_hip_ie)[1]
    joint_vel[15] = pb.getJointState(robot, DracoJointIdx.r_hip_aa)[1]
    joint_vel[16] = pb.getJointState(robot, DracoJointIdx.r_hip_fe)[1]
    joint_vel[17] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[1]
    joint_vel[18] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[1]
    joint_vel[19] = pb.getJointState(robot, DracoJointIdx.r_ankle_fe)[1]
    joint_vel[20] = pb.getJointState(robot, DracoJointIdx.r_ankle_ie)[1]
    # RH
    joint_vel[21] = pb.getJointState(robot, DracoJointIdx.r_shoulder_fe)[1]
    joint_vel[22] = pb.getJointState(robot, DracoJointIdx.r_shoulder_aa)[1]
    joint_vel[23] = pb.getJointState(robot, DracoJointIdx.r_shoulder_ie)[1]
    joint_vel[24] = pb.getJointState(robot, DracoJointIdx.r_elbow_fe)[1]
    joint_vel[25] = pb.getJointState(robot, DracoJointIdx.r_wrist_ps)[1]
    joint_vel[26] = pb.getJointState(robot, DracoJointIdx.r_wrist_pitch)[1]

    # normal force measured on each foot
    _l_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot,
                                   linkIndexA=DracoLinkIdx.l_ankle_ie_link)
    for contact in contacts:
        # add z-component on all points of contact
        _l_normal_force += contact[9]

    _r_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot,
                                   linkIndexA=DracoLinkIdx.r_ankle_ie_link)
    for contact in contacts:
        # add z-component on all points of contact
        _r_normal_force += contact[9]

    b_lf_contact = (True if pb.getLinkState(robot, DracoLinkIdx.l_foot_contact,
                                            1, 1)[0][2] <= 0.05 else False)
    b_rf_contact = (True if pb.getLinkState(robot, DracoLinkIdx.r_foot_contact,
                                            1, 1)[0][2] <= 0.05 else False)
    return (
        imu_frame_quat,
        imu_ang_vel,
        imu_dvel,
        joint_pos,
        joint_vel,
        b_lf_contact,
        b_rf_contact,
        _l_normal_force,
        _r_normal_force,
    )


# TODO:try to modify with setjointmotorcontrol "array" API
def apply_control_input_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL

    # LF
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

    # LH
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

    # neck
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.neck_pitch,
                             controlMode=mode,
                             force=command[13])

    # RF
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

    # RH
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
    pb.resetJointState(robot, DracoJointIdx.l_shoulder_aa, np.pi / 6, 0.0)
    pb.resetJointState(robot, DracoJointIdx.l_elbow_fe, -np.pi / 2, 0.0)
    pb.resetJointState(robot, DracoJointIdx.r_shoulder_aa, -np.pi / 6, 0.0)
    pb.resetJointState(robot, DracoJointIdx.r_elbow_fe, -np.pi / 2, 0.0)

    # Lowerbody
    hip_yaw_angle = 0
    pb.resetJointState(robot, DracoJointIdx.l_hip_aa,
                       np.radians(hip_yaw_angle), 0.0)
    pb.resetJointState(robot, DracoJointIdx.l_hip_fe, -np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.l_knee_fe_jp, np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.l_knee_fe_jd, np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.l_ankle_fe, -np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.l_ankle_ie,
                       np.radians(-hip_yaw_angle), 0.0)

    pb.resetJointState(robot, DracoJointIdx.r_hip_aa,
                       np.radians(-hip_yaw_angle), 0.0)
    pb.resetJointState(robot, DracoJointIdx.r_hip_fe, -np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.r_knee_fe_jp, np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.r_knee_fe_jd, np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.r_ankle_fe, -np.pi / 4, 0.0)
    pb.resetJointState(robot, DracoJointIdx.r_ankle_ie,
                       np.radians(hip_yaw_angle), 0.0)


def signal_handler(signal, frame):
    # if Config.VIDEO_RECORD:
    # pybullet_util.make_video(video_dir, False)
    if Config.MEASURE_COMPUTATION_TIME:
        print("========================================================")
        print('saving list of compuation time in "compuation_time.txt"')
        print("========================================================")
        np.savetxt("computation_time.txt",
                   np.array([compuation_cal_list]),
                   delimiter=",")

    if Config.VIDEO_RECORD:
        print("========================================================")
        print("Making Video")
        print("========================================================")
        pybullet_util.make_video(video_dir)

    pb.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    ## connect pybullet sim server
    pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    pb.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=120,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5],
    )
    ## sim physics setting
    pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                                 numSubSteps=Config.N_SUBSTEP)
    pb.setGravity(0, 0, 0)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    # cylinder_robot = pb.loadURDF(cwd + "/robot_model/draco/draco_modified.urdf",
    # Config.INITIAL_BASE_JOINT_POS,
    # Config.INITIAL_BASE_JOINT_QUAT,
    # useFixedBase=0)
    cylinder_robot = pb.loadURDF(
        cwd + "/robot_model/cylinder.urdf",
        Config.INITIAL_BASE_JOINT_POS,
        Config.INITIAL_BASE_JOINT_QUAT,
        useFixedBase=0,
    )

    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf",
                         useFixedBase=1)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    # TODO:modify this function without dictionary container
    (
        n_q,
        n_v,
        n_a,
        joint_id_dict,
        link_id_dict,
        pos_basejoint_to_basecom,
        rot_basejoint_to_basecom,
    ) = pybullet_util.get_robot_config(
        cylinder_robot,
        Config.INITIAL_BASE_JOINT_POS,
        Config.INITIAL_BASE_JOINT_QUAT,
        Config.PRINT_ROBOT_INFO,
    )

    # robot initial config setting
    # set_init_config_pybullet_robot(cylinder_robot)

    # robot joint and link dynamics setting
    # TODO:modify this function without dictionary container
    pybullet_util.set_joint_friction(cylinder_robot, joint_id_dict, 0)
    pybullet_util.set_link_damping(cylinder_robot, link_id_dict, 0.0, 0.0)

    # pnc interface, sensor_data, command class
    rpc_draco_interface = draco_interface_py.DracoInterface()
    # print("\nDONE REACHED\n")
    rpc_draco_sensor_data = draco_interface_py.DracoSensorData()
    rpc_draco_command = draco_interface_py.DracoCommand()


    # default robot kinematics information
    base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(
        cylinder_robot)
    rot_world_basecom = util.quat_to_rot(np.array(base_com_quat))
    rot_world_basejoint = util.quat_to_rot(
        np.array(Config.INITIAL_BASE_JOINT_QUAT))

    pos_basejoint_to_basecom = np.dot(
        rot_world_basejoint.transpose(),
        base_com_pos - np.array(Config.INITIAL_BASE_JOINT_POS),
    )
    rot_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                      rot_world_basecom)

    # Run Simulation
    dt = Config.CONTROLLER_DT
    count = 0
    jpg_count = 0

    ## simulation options
    if Config.MEASURE_COMPUTATION_TIME:
        timer = TicToc()
        compuation_cal_list = []

    if Config.VIDEO_RECORD:
        video_dir = "video/draco"
        if os.path.exists(video_dir):
            shutil.rmtree(video_dir)
        os.makedirs(video_dir)

    previous_torso_velocity = np.array([0.0, 0.0, 0.0])
    # rate = RateLimiter(frequency=1. / dt)
    # rate = RateLimiter(frequency=1./5./dt)
    while True:
        l_normal_volt_noise = np.random.normal(0, l_contact_volt_noise)
        r_normal_volt_noise = np.random.normal(0, r_contact_volt_noise)
        imu_ang_vel_noise = np.random.normal(0, imu_ang_vel_noise_std_dev)

        ############################################################
        # Moving Camera Setting
        ############################################################
        base_pos, base_ori = pb.getBasePositionAndOrientation(cylinder_robot)
        pb.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=120,
            cameraPitch=-30,
            cameraTargetPosition=base_pos +
            np.array([0.5, 0.3, -base_pos[2] + 1]),
        )

        ###############################################################################
        # Debugging Purpose
        ##############################################################################
        ##debugging state estimator by calculating groundtruth basejoint states
        base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(
            cylinder_robot)
        rot_world_basecom = util.quat_to_rot(base_com_quat)
        rot_world_basejoint = np.dot(rot_world_basecom,
                                     rot_basejoint_to_basecom.transpose())
        base_joint_pos = base_com_pos - np.dot(rot_world_basejoint,
                                               pos_basejoint_to_basecom)
        base_joint_quat = util.rot_to_quat(rot_world_basejoint)

        base_com_lin_vel, base_com_ang_vel = pb.getBaseVelocity(cylinder_robot)
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

        # pass debugged data to rpc interface
        rpc_draco_sensor_data.base_joint_pos_ = base_joint_pos
        rpc_draco_sensor_data.base_joint_quat_ = base_joint_quat
        rpc_draco_sensor_data.base_joint_lin_vel_ = base_joint_lin_vel
        rpc_draco_sensor_data.base_joint_ang_vel_ = base_joint_ang_vel
        ##############################################################################
        ##############################################################################

        # Get Keyboard Event
        keys = pb.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, "1"):
            rpc_draco_interface.interrupt_.PressOne()
        elif pybullet_util.is_key_triggered(keys, "2"):
            rpc_draco_interface.interrupt_.PressTwo()
        elif pybullet_util.is_key_triggered(keys, "4"):
            rpc_draco_interface.interrupt_.PressFour()
        elif pybullet_util.is_key_triggered(keys, "5"):
            rpc_draco_interface.interrupt_.PressFive()
        elif pybullet_util.is_key_triggered(keys, "6"):
            rpc_draco_interface.interrupt_.PressSix()
        elif pybullet_util.is_key_triggered(keys, "7"):
            rpc_draco_interface.interrupt_.PressSeven()
        elif pybullet_util.is_key_triggered(keys, "8"):
            rpc_draco_interface.interrupt_.PressEight()
        elif pybullet_util.is_key_triggered(keys, "9"):
            rpc_draco_interface.interrupt_.PressNine()

        # get FoxGlove events / commands
        # if Config.USE_FOXGLOVE:
        #     if step_listener.has_been_modified():
        #         if step_listener.is_cmd_triggered('n_steps'):
        #             new_steps_num = step_listener.get_val('n_steps')
        #             rpc_draco_interface.interrupt_.PressStepNum(new_steps_num)
        #             step_listener.reset()

        # get sensor data
        (
            imu_frame_quat,
            imu_ang_vel,
            imu_dvel,
            joint_pos,
            joint_vel,
            b_lf_contact,
            b_rf_contact,
            l_normal_force,
            r_normal_force,
        ) = get_sensor_data_from_pybullet(cylinder_robot)
        l_normal_force = pybullet_util.simulate_contact_sensor(l_normal_force)
        r_normal_force = pybullet_util.simulate_contact_sensor(r_normal_force)
        imu_dvel = pybullet_util.add_sensor_noise(imu_dvel, imu_dvel_bias)
        imu_ang_vel = pybullet_util.add_sensor_noise(imu_ang_vel,
                                                     imu_ang_vel_noise)
        l_normal_force = pybullet_util.add_sensor_noise(
            l_normal_force, l_normal_volt_noise)
        r_normal_force = pybullet_util.add_sensor_noise(
            r_normal_force, r_normal_volt_noise)

        # copy sensor data to rpc sensor data class
        rpc_draco_sensor_data.imu_frame_quat_ = imu_frame_quat
        rpc_draco_sensor_data.imu_ang_vel_ = imu_ang_vel
        rpc_draco_sensor_data.imu_dvel_ = imu_dvel
        rpc_draco_sensor_data.imu_lin_acc_ = imu_dvel / dt
        rpc_draco_sensor_data.joint_pos_ = joint_pos
        rpc_draco_sensor_data.joint_vel_ = joint_vel
        rpc_draco_sensor_data.b_lf_contact_ = b_lf_contact
        rpc_draco_sensor_data.b_rf_contact_ = b_rf_contact
        rpc_draco_sensor_data.lf_contact_normal_ = l_normal_force
        rpc_draco_sensor_data.rf_contact_normal_ = r_normal_force

        ##Debugging

        ##compute control command
        if Config.MEASURE_COMPUTATION_TIME:
            timer.tic()

        rpc_draco_interface.GetCommand(rpc_draco_sensor_data,
                                       rpc_draco_command)

        if Config.MEASURE_COMPUTATION_TIME:
            comp_time = timer.tocvalue()
            compuation_cal_list.append(comp_time)

        # copy command data from rpc command class
        rpc_trq_command = rpc_draco_command.joint_trq_cmd_
        rpc_joint_pos_command = rpc_draco_command.joint_pos_cmd_
        rpc_joint_vel_command = rpc_draco_command.joint_vel_cmd_

        # apply command to pybullet robot
        apply_control_input_to_pybullet(cylinder_robot, rpc_trq_command)

        # lfoot_pos = pybullet_util.get_link_iso(cylinder_robot,
        # save current torso velocity for next iteration
        previous_torso_velocity = pybullet_util.get_link_vel(
            cylinder_robot, link_id_dict["torso_imu"])[3:6]

        # DracoLinkIdx.l_foot_contact)[0:3, 3]
        # rfoot_pos = pybullet_util.get_link_iso(cylinder_robot,
        # DracoLinkIdx.r_foot_contact)[0:3, 3]
        # print("------------------------------------")
        # print(rfoot_pos[1] - lfoot_pos[1])

        # print("trq command printout")
        # print(rpc_trq_command)
        # print("jpos command printout")
        # print(rpc_joint_pos_command)
        # print("jpos command printout")
        # print(rpc_joint_vel_command)

        # Save Image file
        if (Config.VIDEO_RECORD) and (count % Config.RECORD_FREQ == 0):
            camera_data = pb.getDebugVisualizerCamera()
            frame = pybullet_util.get_camera_image_from_debug_camera(
                camera_data, Config.RENDER_WIDTH, Config.RENDER_HEIGHT)
            filename = video_dir + "/step%06d.jpg" % jpg_count
            cv2.imwrite(filename, frame)
            jpg_count += 1

        pb.stepSimulation()  # step simulation
        # rate.sleep()  # while loop rate limiter
        # time.sleep(0.001)

        count += 1
