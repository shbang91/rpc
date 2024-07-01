import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data

import scipy

import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  #include pybind module

import time, math
from collections import OrderedDict
import copy
import signal
import shutil

from loop_rate_limiters import RateLimiter

import cv2

from config.draco.pybullet_simulation import *
from util.python_utils import pybullet_util_rl
from util.python_utils import util
from util.python_utils import liegroup

import draco_interface_py

from config.draco.pybullet_simulation import AlipParams

imu_dvel_bias = np.array([0.0, 0.0, 0.0])
l_contact_volt_noise = 0.001
r_contact_volt_noise = 0.001
imu_ang_vel_noise_std_dev = 0.

import tracemalloc

MEASEURE_TIME = True

if MEASEURE_TIME:
    from pytictoc import TicToc


def set_init_config_pybullet_robot(robot, client=None):
    # Upperbody
    #TODO: make a List of the torques that are 0
    for i in range(1, 36):
        client.resetJointState(robot, i, 0., 0.)

    client.resetJointState(robot, DracoJointIdx.l_shoulder_aa, np.pi / 6, 0.)
    client.resetJointState(robot, DracoJointIdx.l_elbow_fe, -np.pi / 2, 0.)
    client.resetJointState(robot, DracoJointIdx.r_shoulder_aa, -np.pi / 6, 0.)
    client.resetJointState(robot, DracoJointIdx.r_elbow_fe, -np.pi / 2, 0.)
    # Lowerbody
    hip_yaw_angle = 0
    client.resetJointState(robot, DracoJointIdx.l_hip_aa,
                           np.radians(hip_yaw_angle), 0.)
    client.resetJointState(robot, DracoJointIdx.l_hip_fe, -np.pi / 4, 0.)
    client.resetJointState(robot, DracoJointIdx.l_knee_fe_jp, np.pi / 4, 0.)

    client.resetJointState(robot, DracoJointIdx.l_knee_fe_jd, np.pi / 4, 0.)
    client.resetJointState(robot, DracoJointIdx.l_ankle_fe, -np.pi / 4, 0.)
    client.resetJointState(robot, DracoJointIdx.l_ankle_ie,
                           np.radians(-hip_yaw_angle), 0.)

    client.resetJointState(robot, DracoJointIdx.r_hip_aa,
                           np.radians(-hip_yaw_angle), 0.)
    client.resetJointState(robot, DracoJointIdx.r_hip_fe, -np.pi / 4, 0.)
    client.resetJointState(robot, DracoJointIdx.r_knee_fe_jp, np.pi / 4, 0.)
    client.resetJointState(robot, DracoJointIdx.r_knee_fe_jd, np.pi / 4, 0.)
    client.resetJointState(robot, DracoJointIdx.r_ankle_fe, -np.pi / 4, 0.)
    client.resetJointState(robot, DracoJointIdx.r_ankle_ie,
                           np.radians(hip_yaw_angle), 0.)


def print_command(rpc_command):

    print("pos cmd", rpc_command.joint_pos_cmd_)
    print("joint vel cmd", rpc_command.joint_vel_cmd_)
    print("joint acc cmd", rpc_command.joint_trq_cmd_)


def print_sensor_data(data):
    print("imu sens", data.imu_frame_quat_)
    print("imu ang sens", data.imu_ang_vel_)
    print("imu dvel", data.imu_dvel_)
    print("imu lin acc sens", data.imu_lin_acc_)
    print("joint pos", data.joint_pos_)
    print("joint vel", data.joint_vel_)
    print("lf contact ", data.b_lf_contact_)
    print("rf contact", data.b_rf_contact_)
    print("lf contact normal", data.lf_contact_normal_)
    print("rf contact normal", data.rf_contact_normal_)


def dict_to_numpy(obs_dict):
    obs = []
    for k, v in obs_dict.items():
        if isinstance(v, dict):
            for k2, v2 in v.items():
                obs.append(v2)
        elif isinstance(v, np.ndarray) or isinstance(v, list) or isinstance(
                v, tuple):
            for i in range(len(v)):
                obs.append(v[i])
        else:
            obs.append(v)
    return np.array(obs)


class DracoEnv(gym.Env):
    metadata = {
        "render.modes": ["human", "rgb_array"],
        "video.frames_per_second": 50
    }

    def __init__(self,
                 Lx_offset_des,
                 Ly_des,
                 yaw_des,
                 mpc_freq,
                 sim_dt,
                 randomized_command: bool = False,
                 reduced_obs_size: bool = False,
                 render: bool = False) -> None:
        self.render = render
        self._reduced_obs_size = reduced_obs_size
        self._randomized_command = randomized_command
        #if randomized command = false, desired will be the command
        #if randomized command = true,  +-desired are the limits of the probability distribution centered around 0
        self._Lx_offset_des = Lx_offset_des
        self._Ly_des = Ly_des
        self._yaw_des = yaw_des
        self._mpc_freq = mpc_freq
        self._sim_dt = sim_dt
        assert Config.CONTROLLER_DT == sim_dt

        if self.render:
            self.client = bc.BulletClient(connection_mode=p.GUI)
            self.client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.client = bc.BulletClient(connection_mode=p.DIRECT)

        #Action definition
        self.action_space = gym.spaces.Box(low=np.array([-1, -1, -1]),
                                           high=np.array([1, 1, 1]),
                                           dtype=np.float64)

        if self._reduced_obs_size:
            self.observation_space = gym.spaces.Box(  #observation space added Tr and previous full_action x and y
                low=np.array([-50] * 19),
                high=np.array([50] * 19),
                dtype=np.float64)
            """
            obs_space = {
                'stance_foot': gym.spaces.Discrete(2),
                'COM' : gym.spaces.Box(low = np.array([-60]*15),
                                       high = np.array([60]*15),
                                       dtype = np.float64)
            }
            """
        else:
            self.observation_space = gym.spaces.Box(  #observation space
                low=np.array([-100] * 74),
                high=np.array([100] * 74),
                dtype=np.float64)
            """
            obs_space = {
                'stance_foot': gym.spaces.Discrete(2),
                'joint_pos_vel': gym.spaces.Box(low = np.array([-60]*52),
                                                high = np.array([60]*52),
                                                dtype = np.float32),
                'COM': gym.spaces.Box(low = np.array([-60]*15),
                                       high = np.array([60]*15),
                                       dtype = np.float32)
            }
            """
            #COM
            #Lx, Ly, des_com_yaw, compos xyz, L xyz, torso rpw, torso ang vel, stance_leg
            #1+1+1+3+3+3+3+1 = 16

        #self.observation_space = gym.spaces.Dict(obs_space)

        if (self.render):
            self.client.resetDebugVisualizerCamera(
                cameraDistance=1.0,
                cameraYaw=120,
                cameraPitch=-30,
                cameraTargetPosition=[1, 0.5, 1.0])
        self.client.setPhysicsEngineParameter(
            fixedTimeStep=Config.CONTROLLER_DT, numSubSteps=Config.N_SUBSTEP)
        self.client.setGravity(0, 0, -9.81)

        self.client.configureDebugVisualizer(self.client.COV_ENABLE_RENDERING,
                                             0)

        # Create Robot, Ground
        self.client.configureDebugVisualizer(self.client.COV_ENABLE_RENDERING,
                                             0)
        self.robot = self.client.loadURDF(
            cwd + "/robot_model/draco/draco_modified.urdf",
            Config.INITIAL_BASE_JOINT_POS,
            Config.INITIAL_BASE_JOINT_QUAT,
            useFixedBase=0,
            flags=p.URDF_USE_SELF_COLLISION)

        ground = self.client.loadURDF(cwd + "/robot_model/ground/plane.urdf",
                                      useFixedBase=1)

        self.client.configureDebugVisualizer(self.client.COV_ENABLE_RENDERING,
                                             1)

        set_init_config_pybullet_robot(self.robot, self.client)

        nq, nv, na, self.joint_id, self.link_id_dict, self.pos_basejoint_to_basecom, self.rot_basejoint_to_basecom = pybullet_util_rl.get_robot_config(
            self.robot,
            Config.INITIAL_BASE_JOINT_POS,
            Config.INITIAL_BASE_JOINT_QUAT,
            Config.PRINT_ROBOT_INFO,
            client=self.client)

        pybullet_util_rl.set_joint_friction(self.robot,
                                            self.joint_id,
                                            0.,
                                            client=self.client)
        pybullet_util_rl.set_link_damping(self.robot,
                                          self.link_id_dict,
                                          0.,
                                          0.,
                                          client=self.client)

        ## rolling contact joint constraint
        c1 = self.client.createConstraint(self.robot,
                                          DracoLinkIdx.l_knee_fe_lp,
                                          self.robot,
                                          DracoLinkIdx.l_knee_fe_ld,
                                          jointType=self.client.JOINT_GEAR,
                                          jointAxis=[0, 1, 0],
                                          parentFramePosition=[0, 0, 0],
                                          childFramePosition=[0, 0, 0])
        self.client.changeConstraint(c1, gearRatio=-1, maxForce=500, erp=10)

        c2 = self.client.createConstraint(self.robot,
                                          DracoLinkIdx.r_knee_fe_lp,
                                          self.robot,
                                          DracoLinkIdx.r_knee_fe_ld,
                                          jointType=self.client.JOINT_GEAR,
                                          jointAxis=[0, 1, 0],
                                          parentFramePosition=[0, 0, 0],
                                          childFramePosition=[0, 0, 0])
        self.client.changeConstraint(c2, gearRatio=-1, maxForce=500, erp=10)
        #pnc interface, sensor_data, command class

        self._rpc_draco_interface = draco_interface_py.DracoInterface()
        self._rpc_draco_sensor_data = draco_interface_py.DracoSensorData()
        self._rpc_draco_command = draco_interface_py.DracoCommand()

        #reward terms
        self._w_roll_pitch = -1
        self._w_com_height = -1
        self._w_desired_Lxy = 1.
        self._w_desired_Lx = 2.
        self._w_desired_Ly = 3.
        self._w_desired_yaw = -1.
        self._w_excessive_fp = -0.5
        self._w_excessive_angle = -0.5
        self._w_termination = -20.
        self._w_alive_bonus = 3.
        self._w_action_diff = -0.001
        self._w_penalise_trajectory_Lx = -0.0005
        self._w_penalise_trajectory_Ly = -0.0005

        self._Lx_main = 0.5*AlipParams.WIDTH*AlipParams.MASS*math.sqrt(AlipParams.G/AlipParams.ZH) \
                        *AlipParams.ZH*math.tanh(math.sqrt(AlipParams.G/AlipParams.ZH)*AlipParams.TS/2)

    def reset(self, seed: int = 0):  #creates env
        # Environment Setup
        # self.client.resetSimulation()

        self._rpc_draco_interface.Reset()

        self._DELETE()

        #initialise old_wbc_obs for reward
        self._old_wbc_obs = np.zeros(20)
        self._new_wbc_obs = np.zeros(20)

        self._rpc_draco_sensor_data = draco_interface_py.DracoSensorData()
        self._rpc_draco_command = draco_interface_py.DracoCommand()

        self.client.configureDebugVisualizer(self.client.COV_ENABLE_RENDERING,
                                             1)

        self.client.resetBasePositionAndOrientation(
            self.robot, [-0.031658, -3.865e-5, 0.88019], [0., 0., 0., 1.])

        self.client.resetBaseVelocity(self.robot, [0., 0., 0.], [0., 0., 0.])
        set_init_config_pybullet_robot(self.robot, self.client)

        nq, nv, na, self.joint_id, self.link_id_dict, self.pos_basejoint_to_basecom, self.rot_basejoint_to_basecom = pybullet_util_rl.get_robot_config(
            self.robot,
            Config.INITIAL_BASE_JOINT_POS,
            Config.INITIAL_BASE_JOINT_QUAT,
            Config.PRINT_ROBOT_INFO,
            client=self.client)

        base_com_pos, base_com_quat = self.client.getBasePositionAndOrientation(
            self.robot)
        rot_world_basecom = util.quat_to_rot(np.array(base_com_quat))
        rot_world_basejoint = util.quat_to_rot(
            np.array(Config.INITIAL_BASE_JOINT_QUAT))

        pos_basejoint_to_basecom = np.dot(
            rot_world_basejoint.transpose(),
            base_com_pos - np.array(Config.INITIAL_BASE_JOINT_POS))
        rot_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                          rot_world_basecom)

        # Run Simulation
        self.dt = Config.CONTROLLER_DT

        self.previous_torso_velocity = np.array([0., 0., 0.])
        self.rate = RateLimiter(frequency=1. / self.dt)

        self.set_action_command_in_sensor_data()

        self._debug_sensor_data()

        obs_numpy = self._get_observation(self._rpc_draco_command.wbc_obs_)

        info = {
            "interface": self._rpc_draco_interface,
        }
        self._iter = 0

        #ACTION COMMAND WILL CHANGE ONCE PER EPISODE NOT DURING STEPS

        return obs_numpy, info

    def step(self, action):
        #residual, self.gripper_command = action[0], action[1]
        # TODO remove printing
        step_flag = False
        while not step_flag:
            l_normal_volt_noise = np.random.normal(0, l_contact_volt_noise)
            r_normal_volt_noise = np.random.normal(0, r_contact_volt_noise)
            imu_ang_vel_noise = np.random.normal(0, imu_ang_vel_noise_std_dev)

            if self._debug_sensor_data():
                done = True
                break

            wbc_action = self._denormalise_action(action)

            self.pybulled_to_sensor_data(wbc_action)

            self._rpc_draco_interface.GetCommand(self._rpc_draco_sensor_data,
                                                 self._rpc_draco_command)
            step_flag = self._rpc_draco_command.rl_trigger_

            self._set_motor_command(self._rpc_draco_command, self.client)

            self.previous_torso_velocity = pybullet_util_rl.get_link_vel(
                self.robot, self.link_id_dict['torso_imu'], self.client)[3:6]

            rand_num = np.random.randint(0, 800)
            print(rand_num)
            if rand_num == 0:
                rand_num = np.random.randint(0, 2)
                rand_force = np.zeros(3)
                if rand_num == 0: rand_force[0] = 5000
                elif rand_num == 1: rand_force[1] = 5000
                self.client.applyExternalForce(self.robot,
                                               -1,
                                               rand_force,
                                               np.zeros(3),
                                               flags=self.client.WORLD_FRAME)

            self.client.stepSimulation()
            if self.render: self.rate.sleep()
            done = self._compute_termination(self._rpc_draco_command.wbc_obs_)
            if done: break

        policy_obs = self._get_observation(self._rpc_draco_command.wbc_obs_)

        if (self._iter > 30 * 48): truncate = True
        else: truncate = False

        self._iter += 1

        wbc_action = self._denormalise_action(action)

        reward = self._compute_reward(self._rpc_draco_command.wbc_obs_,
                                      wbc_action, done)

        info = {"reward_components": self.reward_info}
        #self.dataf
        return policy_obs, reward, done, truncate, info  # need terminated AND truncated

    def _denormalise_action(self, action):
        #from -1 to 1 to-0.1 to 1
        #for now dummy
        wbc_action = 0.1 * action
        return wbc_action

    def set_action_command_in_sensor_data(self):
        #maybe set also time in newer version
        if self._randomized_command:
            #prob dist
            limit = np.array([
                np.abs(self._Lx_offset_des),
                np.abs(self._Ly_des),
                np.abs(self._yaw_des)
            ])
            dir_command = np.random.uniform(low=-limit, high=limit)
        else:
            dir_command = np.array(
                (self._Lx_offset_des, self._Ly_des, self._yaw_des))

        initial_stance_leg = np.random.choice(np.array([-1, 1]))

        self._rpc_draco_sensor_data.initial_stance_leg_ = initial_stance_leg
        self._rpc_draco_sensor_data.policy_command_ = dir_command

    def _set_motor_command(self, command, client) -> None:
        rpc_trq_command = command.joint_trq_cmd_

        pybullet_util_rl.apply_control_input_to_pybullet(
            self.robot, rpc_trq_command, DracoJointIdx, client)

    def pybulled_to_sensor_data(self, action):
        imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, b_rf_contact, \
            l_normal_force, r_normal_force = pybullet_util_rl.get_sensor_data_from_pybullet(
            self.robot, DracoLinkIdx, DracoJointIdx, self.previous_torso_velocity,
            self.link_id_dict, self.client)

        l_normal_force = pybullet_util_rl.simulate_contact_sensor(
            l_normal_force)
        r_normal_force = pybullet_util_rl.simulate_contact_sensor(
            r_normal_force)

        self._rpc_draco_sensor_data.imu_frame_quat_ = imu_frame_quat
        self._rpc_draco_sensor_data.imu_ang_vel_ = imu_ang_vel
        self._rpc_draco_sensor_data.imu_dvel_ = imu_dvel
        self._rpc_draco_sensor_data.imu_lin_acc_ = imu_dvel / self.dt
        self._rpc_draco_sensor_data.joint_pos_ = joint_pos
        self._rpc_draco_sensor_data.joint_vel_ = joint_vel
        self._rpc_draco_sensor_data.b_lf_contact_ = b_lf_contact
        self._rpc_draco_sensor_data.b_rf_contact_ = b_rf_contact
        self._rpc_draco_sensor_data.lf_contact_normal_ = l_normal_force
        self._rpc_draco_sensor_data.rf_contact_normal_ = r_normal_force
        self._rpc_draco_sensor_data.res_rl_action_ = action

    def _get_observation(self, wbc_obs) -> dict:
        """
        if self._reduced_obs_size:
            if wbc_obs is None:
                wbc_obs = np.array([self._rpc_draco_interface.action_command_[0], self._rpc_draco_interface.action_command_[1], self._rpc_draco_interface.action_command_[2],
                                    self._rpc_draco_interface.action_command_[3], 0., 0., 0., 0.,0.,0., 0.,0.,0.,0.,0.,0.])

            policy_obs = np.concatenate((wbc_obs[0:12], wbc_obs[15:18]))

        else:
            imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, b_rf_contact, \
                l_normal_force, r_normal_force = pybullet_util_rl.get_sensor_data_from_pybullet(
                self.robot, DracoLinkIdx, DracoJointIdx, self.previous_torso_velocity,
                self.link_id_dict, self.client)
            policy_obs = np.concatenate((joint_pos, joint_vel))
            if wbc_obs is None:
                wbc_obs = np.array([AlipParams.INITIAL_STANCE_LEG, AlipParams.LX_OFFSET, AlipParams.Ly_des_,
                                    AlipParams.COM_YAW, 0, 0, 0, 0,0,0])                                       #Ts removed
                policy_obs = np.concatenate((joint_pos, joint_vel))
            else:
                policy_obs = np.concatenate((joint_pos, joint_vel, wbc_obs[0:13]))

        """
        stance_leg = np.array([int(wbc_obs[0])])
        #if stance_leg == -1: stance_leg = 0
        COM = np.concatenate((wbc_obs[1:10], wbc_obs[13:16],
                              self._rpc_draco_sensor_data.base_joint_ang_vel_,
                              stance_leg, wbc_obs[16:20]))
        COM[16] -= self._sim_dt
        if (self._reduced_obs_size):
            """
            policy_obs = {'stance_foot': stance_leg,
                          'COM': COM}
            """
            policy_obs = COM
        else:
            imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, b_rf_contact, \
                l_normal_force, r_normal_force = pybullet_util_rl.get_sensor_data_from_pybullet(
                self.robot, DracoLinkIdx, DracoJointIdx, self.previous_torso_velocity,
                self.link_id_dict, self.client)
            joint_obs = np.concatenate((joint_pos, joint_vel))
            """
            policy_obs = {'stance_foot': stance_leg,
                          'joit_pos_vel': joint_obs,
                          'COM': COM}
            """
            policy_obs = np.concatenate((joint_obs, COM))

        return policy_obs

    def _compute_termination(self, _wbc_obs=None):
        #TODO: add more termination
        if _wbc_obs is not None:
            #condition = np.any((_wbc_obs[6] < 0.5) | (_wbc_obs[6] > 0.8))  #0.69
            if _wbc_obs[6] > 0.75:
                return True
            if _wbc_obs[6] < 0.57:
                return True
            if np.abs(
                    _wbc_obs[7]) > (np.abs(self._Lx_main + _wbc_obs[1]) + 25):
                return True
            if np.abs(_wbc_obs[8] - _wbc_obs[2]) > 25:
                return True
        return False

    def _compute_reward(self, wbc_obs, action, done):
        if (done):
            self.reward_info = -10
            return -10
        if wbc_obs is None: return 0
        self._old_wbc_obs = np.copy(self._new_wbc_obs)
        self._new_wbc_obs = np.copy(wbc_obs)
        self._rl_action = np.copy(action)

        if (self._old_wbc_obs[0] != self._new_wbc_obs[0]):

            reward = self._w_alive_bonus
            #reward += self.reward_tracking_com_L()
            reward += self.reward_tracking_com_Lx()
            reward += self.reward_tracking_com_Ly()
            reward += self.reward_tracking_yaw()
            reward += self.reward_com_height()
            reward += self.reward_roll_pitch()
            reward += self.penalise_excessive_fp()
            reward += self.penalise_excessive_yaw()
            #if done: reward -= self._w_termination
            self.reward_info = np.array([
                self._w_alive_bonus,
                self.reward_tracking_com_L(),
                self.reward_tracking_yaw(),
                self.reward_com_height(),
                self.reward_roll_pitch(),
                self.penalise_excessive_fp(),
                self.penalise_excessive_yaw()
            ])
        else:  #intrastep reward
            reward = self.penalise_different_policy()
            reward += self.penalise_excessive_Lx()
            reward += self.penalise_excessive_Ly()
            self.reward_info = np.array([
                self.penalise_different_policy(),
                self.penalise_excessive_Lx(),
                self.penalise_excessive_Ly()
            ])

        return reward.item()

    def reward_tracking_com_L(self):
        L = np.zeros(2)
        if (self._new_wbc_obs[0] == 1):
            L[0] = self._old_wbc_obs[1] + self._Lx_main  #Lx_offset+ LX_MAIN
        else:
            L[0] = self._old_wbc_obs[1] - self._Lx_main
        #in the code 1 corresponds to current stance foot right
        # -1 to current stance foot left
        # new obs -1 --> ended policy for left foot --> we are at the desired state for end of right stance
        error = L - self._new_wbc_obs[
            7:
            9]  #+ self._old_wbc_obs[1:3] - self._new_wbc_obs[9:11]  #desired Lx,y - observedLx,y at the end of the step
        error = np.sum(np.square(error))
        error = np.exp(-error * 0.1)

        error *= self._w_desired_Lxy
        return error

    def reward_tracking_com_Lx(self):
        if (self._new_wbc_obs[0] == 1):
            L = self._old_wbc_obs[1] + self._Lx_main  #Lx_offset+ LX_MAIN
        else:
            L = self._old_wbc_obs[1] - self._Lx_main
        #in the code 1 corresponds to current stance foot right
        # -1 to current stance foot left
        # new obs -1 --> ended policy for left foot --> we are at the desired state for end of right stance
        error = L - self._new_wbc_obs[
            7]  #+ self._old_wbc_obs[1:3] - self._new_wbc_obs[9:11]  #desired Lx,y - observedLx,y at the end of the step
        error = np.square(error)
        error = np.exp(-error * 0.1)

        error *= self._w_desired_Lx
        return error

    def reward_tracking_com_Ly(self):
        error = self._old_wbc_obs[2] - self._new_wbc_obs[8]
        error = np.square(error)
        error = np.exp(-error * 0.1)

        error *= self._w_desired_Ly
        return error

    def reward_tracking_yaw(self):
        error = self._new_wbc_obs[15] - self._old_wbc_obs[
            15] - self._old_wbc_obs[3]
        #error = np.square(error)
        #eror = np.exp(-error)
        error = np.abs(error)
        error *= self._w_desired_yaw

        return error

    def reward_com_height(self):
        error = self._new_wbc_obs[6] - AlipParams.ZH
        #error = np.square(error)
        error = np.abs(error)

        error *= self._w_com_height
        return error

    def reward_roll_pitch(self):
        #error = np.sum(np.square(self._new_wbc_obs[15:17]))
        #error = np.exp(-error)
        error = scipy.linalg.norm(self._new_wbc_obs[13:15])
        error *= self._w_roll_pitch
        return error

    def penalise_excessive_fp(self):
        #error = np.sum(np.square(self._rl_action[0:2]))
        #error = np.exp(-error)
        error = scipy.linalg.norm(self._rl_action[0:2])

        error *= self._w_excessive_fp
        return error

    def penalise_excessive_yaw(self):
        #rror = np.square(self._rl_action[2])
        #error = np.exp(-error)
        error = np.abs(self._rl_action[2])
        error *= self._w_excessive_angle

        return error

    def _debug_sensor_data(self):
        ###############################################################################
        #Debugging Purpose
        ##############################################################################
        ##debugging state estimator by calculating groundtruth basejoint states
        base_com_pos, base_com_quat = self.client.getBasePositionAndOrientation(
            self.robot)
        done = False
        if np.isnan(base_com_quat).any() or np.isinf(base_com_quat).any():
            done = True
            return True
        norm = scipy.linalg.norm(base_com_quat)
        if (norm == 0):
            done = True
            self.reset()

        if done:
            return True

        rot_world_basecom = util.quat_to_rot(base_com_quat)
        rot_world_basejoint = np.dot(rot_world_basecom,
                                     self.rot_basejoint_to_basecom.transpose())
        base_joint_pos = base_com_pos - np.dot(rot_world_basejoint,
                                               self.pos_basejoint_to_basecom)
        base_joint_quat = util.rot_to_quat(rot_world_basejoint)

        base_com_lin_vel, base_com_ang_vel = self.client.getBaseVelocity(
            self.robot)
        trans_joint_com = liegroup.RpToTrans(self.rot_basejoint_to_basecom,
                                             self.pos_basejoint_to_basecom)
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
        self._rpc_draco_sensor_data.base_joint_pos_ = base_joint_pos
        self._rpc_draco_sensor_data.base_joint_quat_ = base_joint_quat
        self._rpc_draco_sensor_data.base_joint_lin_vel_ = base_joint_lin_vel
        self._rpc_draco_sensor_data.base_joint_ang_vel_ = base_joint_ang_vel
        #########################
        #### DEBUGGING   END ####
        #########################
        return False

    def close(self):
        self.client.disconnect()
        self.client = None

    def _DELETE(self):
        try:
            del self._rpc_draco_sensor_data
        except AttributeError:
            pass

        try:
            del self._rpc_draco_command
        except AttributeError:
            pass

        try:
            del self.pos_basejoint_to_basecom
        except AttributeError:
            pass

        try:
            del self.rot_basejoint_to_basecom
        except AttributeError:
            pass

        try:
            del self._iter
        except AttributeError:
            pass

        try:
            del self.previous_torso_velocity
        except AttributeError:
            pass

        try:
            del self._old_wbc_obs
        except AttributeError:
            pass

        try:
            del self._new_wbc_obs
        except AttributeError:
            pass

        try:
            del self._rl_action
        except AttributeError:
            pass


if __name__ == "__main__":
    yaw_max = math.pi / 4
    env = DracoEnv(0.,
                   0.,
                   0.,
                   5,
                   Config.CONTROLLER_DT,
                   randomized_command=False,
                   reduced_obs_size=False,
                   render=True)
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    obs, info = env.reset()
    interface = info["interface"]
    iter = 0
    flag = False

    while True:
        action = np.zeros(3)
        obs, reward, done, trunc, info = env.step(action)
        print(info['reward_components'])
        if done or trunc:
            obs, info = env.reset()
        if flag:
            flag = False
            obs, info = env.reset()
