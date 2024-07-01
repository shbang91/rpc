import gymnasium as gym
import numpy as np
import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from util.python_utils import pybullet_util_rl
from config.draco.pybullet_simulation import *

from simulator.pybullet.rl.env_2 import *
import random


class DracoEnvMpcFreq_Lx_range_r_intra_pol_variance_0p005(DracoEnv_v2):

    def __init__(self,
                 mpc_freq,
                 sim_dt,
                 eval=None,
                 burn_in: bool = False,
                 reduced_obs_size: bool = False,
                 render: bool = False,
                 disturbance: bool = False) -> None:
        super().__init__(mpc_freq=mpc_freq,
                         sim_dt=sim_dt,
                         reduced_obs_size=reduced_obs_size,
                         render=render,
                         eval=eval,
                         disturbance=disturbance)

        self._burn_in = burn_in
        if mpc_freq == 0:
            print("FREQ SET TO 0. PLEASE INCREASE FREQ")
            raise Warning

        self._set_max_steps_iter(32 * 150)

        self._freq_push_dict = {
            'long_push_x': [572, 10, 0],
            'short_push_x': [6, 80, 0],
            'long_push_y': [572, 0, 10],
            'short_push_y': [6, 0, 100]
        }
        self._push_trigger = 2000
        self._push_ = [-1, -1, -1]
        self._COUNTER = 0

        #raise Warning

    def _set_observation_space(self):
        if self._reduced_obs_size:
            self.observation_space = gym.spaces.Box(  #observation space added Tr and previous full_action x and y
                low=np.array([-100] * 21),
                high=np.array([100] * 21),
                dtype=np.float64)
        else:
            self.observation_space = gym.spaces.Box(  #observation space
                low=np.array([-100] * 75),
                high=np.array([100] * 75),
                dtype=np.float64)

    def _get_observation(self, wbc_obs) -> dict:
        """ Desired state is not an input to nn implicit in the env
        stance_leg
        com_pos_desired_frame x3
        L_desired_frame       x3
        torso_roll_pitch_yaw  x3
        swfoot_roll_pitch_yaw x3
        torso_ang_vel         x3
        Tr                    x1
        last policy           x3

        """
        COM = np.concatenate((np.array([wbc_obs[0]]), wbc_obs[4:10],
                              wbc_obs[13:19], wbc_obs[24:27], wbc_obs[19:23]))
        COM[16] -= self._sim_dt
        if (self._reduced_obs_size):
            policy_obs = COM
        else:
            imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, b_rf_contact, \
                l_normal_force, r_normal_force = pybullet_util_rl.get_sensor_data_from_pybullet(
                self.robot, DracoLinkIdx, DracoJointIdx, self.previous_torso_velocity,
                self.link_id_dict, self.client)
            joint_obs = np.concatenate((joint_pos, joint_vel))
            policy_obs = np.concatenate((joint_obs, COM))

        policy_obs = np.concatenate((policy_obs, [self._Lx]), axis=0)

        return policy_obs

    def _normalise_action(self, action):
        _wbc_action = 0.15 * action
        _wbc_action[2] = 0.05 * action[2]
        if self._burn_in:
            _wbc_action = 0 * _wbc_action
        return _wbc_action

    def _compute_termination(self, _wbc_obs=None):
        if np.abs(_wbc_obs[23] - 12) < 0.5:  #12 is the alip state
            if _wbc_obs is not None:
                #condition = np.any((_wbc_obs[6] < 0.5) | (_wbc_obs[6] > 0.8))  #0.69
                if _wbc_obs[6] > 1:
                    return True
                if _wbc_obs[6] < 0.45:
                    return True
                if np.abs(_wbc_obs[7]) > (np.abs(self._Lx_main + _wbc_obs[1]) +
                                          100):

                    return True
                if np.abs(_wbc_obs[8] - _wbc_obs[2]) > 50:

                    return True
        return False

    def set_action_command_in_sensor_data(self):
        #maybe set also time in newer version
        # Lx_list = [-30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 18, 20, 25, 30]
        Lx_list = [x for x in range(-25, 26, 5)]
        # Lx_list = [-8.5, -7, -5, -3, -1, 0, 1, 3, 5, 7, 8.5]
        self._Lx = random.choice(Lx_list)
        dir_command = np.array((self._Lx, 0, 0))
        initial_stance_leg = np.random.choice(np.array([-1, 1]))

        self._rpc_draco_sensor_data.initial_stance_leg_ = initial_stance_leg
        self._rpc_draco_sensor_data.policy_command_ = dir_command

    def _set_reward_coeffs(self):
        #reward terms
        self._w_roll_pitch = -1
        self._w_com_height = -1
        self._w_penalise_excessive_Lx = 0.8
        self._w_desired_Lx = 2.5
        self._w_Lx_offset = 1.5

        self._w_desired_Ly = 2
        self._w_desired_yaw = 1.
        self._w_excessive_fp = -1
        self._w_excessive_angle = -3
        self._w_termination = 0
        self._w_alive_bonus = 0.3
        self._w_intra_pol = 0.2  ## TODO: this has only been changed here
        self._w_intra_Lx = 0.01
        self._w_intra_Ly = 0.005

    def _set_push_trigger(self):
        return 3000000

    def _compute_reward(self, wbc_obs, action, done):
        if (done):
            self.reward_info = self._w_termination
            #return self._w_termination/self._iter
            return self._w_termination
        if wbc_obs is None: return 0

        self._old_wbc_obs = np.copy(self._new_wbc_obs)
        self._new_wbc_obs = np.copy(wbc_obs)
        self._rl_action = np.copy(action)

        if (self._old_wbc_obs[0] != self._new_wbc_obs[0]):
            reward = self._w_alive_bonus
            reward += self.reward_tracking_com_Lx()
            reward += self.penalise_outside_Lx_bounds()
            #reward += self.tracking_Lx_offset()
            reward += self.reward_tracking_com_Ly()
            reward += self.reward_tracking_yaw()
            reward += self.reward_com_height()
            reward += self.reward_roll_pitch()
            reward += self.penalise_excessive_fp()
            reward += self.penalise_excessive_yaw()
            #if done: reward -= self._w_termination
            self.reward_info = np.array([
                reward, self._w_alive_bonus,
                self.reward_tracking_com_Lx(),
                self.penalise_outside_Lx_bounds(),
                self.reward_tracking_com_Ly(),
                self.reward_tracking_yaw(),
                self.reward_com_height(),
                self.reward_roll_pitch(),
                self.penalise_excessive_fp(),
                self.penalise_excessive_yaw()
            ])
            self._COUNTER += 1
            if (self._COUNTER >= 20):
                self.set_action_command_in_sensor_data()
                self._COUNTER = 0

        else:
            reward = self.r_intra_different_policy()
            reward += self.r_intra_excessive_Lx()
            reward += self.r_intra_excessive_Ly()
            self.reward_info = np.array([
                self.r_intra_different_policy(),
                self.r_intra_excessive_Lx(),
                self.r_intra_excessive_Ly()
            ])
        return reward.item()

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
        error /= (self._mass * self._zH)

        error = np.square(error / 0.1)
        error = np.exp(-error)

        error *= self._w_desired_Lx
        return error

    def penalise_outside_Lx_bounds(self):
        error = np.abs(self._new_wbc_obs[7] -
                       self._old_wbc_obs[1]) - self._Lx_main
        error /= (self._mass * self._zH)

        if error < 0:
            return self._w_penalise_excessive_Lx
        error = np.square(error)
        error = np.exp(-error / 0.01)
        return error

    def tracking_Lx_offset(self):
        error = self._new_wbc_obs[7] - self._old_wbc_obs[1]
        error /= (self._mass * self._zH)
        error = np.square(error)
        #error = np.exp(-error/0.03)
        error = np.exp(-error / 0.1)
        return self._w_Lx_offset * error

    def reward_tracking_com_Ly(self):
        error = self._old_wbc_obs[2] - self._new_wbc_obs[8]
        error /= (self._mass * self._zH)

        error = np.square(error)
        error = np.exp(-error / 0.01)

        error *= self._w_desired_Ly
        return error

    def reward_tracking_yaw(self):
        error = self._new_wbc_obs[15] - self._old_wbc_obs[
            15] - self._old_wbc_obs[3]
        #error = np.square(error)
        #eror = np.exp(-error)
        error = np.square(error)
        error = np.exp(-error / 0.01)
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
        error = np.sum(np.square(self._rl_action[0:2]))
        #error = np.exp(-error)
        #error = scipy.linalg.norm(self._rl_action[0:2])

        error *= self._w_excessive_fp
        return error

    def penalise_excessive_yaw(self):
        error = np.square(self._rl_action[2])
        #error = np.exp(-error)
        #error = np.abs(self._rl_action[2])
        error *= self._w_excessive_angle

        return error

    def r_intra_different_policy(self):
        error = self._new_wbc_obs[17:20] - self._old_wbc_obs[17:20]
        error /= self._sim_dt * self._mpc_freq
        error = np.sum(np.square(error))
        error = np.exp(-error / 0.005)
        return self._w_intra_pol * error

    def r_intra_excessive_Lx(self):
        error = np.abs(self._new_wbc_obs[7] -
                       self._old_wbc_obs[1]) - self._Lx_main
        error /= (self._mass * self._zH)

        if error < 0:
            return self._w_intra_Lx

        error = np.exp(-error / 0.01)
        return self._w_intra_Lx * error

    def r_intra_excessive_Ly(self):
        error = self._old_wbc_obs[2] - self._new_wbc_obs[8]
        error /= (self._mass * self._zH)
        error = np.square(error)
        error = np.exp(-error / 0.04)
        return self._w_intra_Ly * error

    def apply_disturbance(self):
        #print("dfa")
        self._push_trigger -= 1
        #print(self._push_trigger)
        if self._push_trigger == 0:
            choice = np.random.randint(0, 4)
            if choice == 0:
                print("short_push x ")
                self._push_ = copy.deepcopy(
                    self._freq_push_dict['short_push_x'])
            elif choice == 1:
                print("short_push y")

                self._push_ = copy.deepcopy(
                    self._freq_push_dict['short_push_y'])
            elif choice == 2:
                print("long_push x ")

                self._push_ = copy.deepcopy(
                    self._freq_push_dict['long_push_x'])
            else:
                print("long_push y")

                self._push_ = copy.deepcopy(
                    self._freq_push_dict['long_push_y'])

            self._push_dir = np.random.choice([-1, 1])
        if self._push_[0] > 0:
            self._push_[0] -= 1
            force = np.array((self._push_[1], self._push_[2], 0))
            force *= self._push_dir
            #print(force)
            self.client.applyExternalForce(self.robot,
                                           1,
                                           force,
                                           np.zeros(3),
                                           flags=self.client.WORLD_FRAME)
            #print("push")
            if self._push_[0] == 0: self._push_trigger = 3000


if __name__ == "__main__":
    env = DracoEnvMpcFreq_Lx_range(5,
                                   Config.CONTROLLER_DT,
                                   reduced_obs_size=True,
                                   render=True)
    from stable_baselines3.common.env_checker import check_env
    check_env(env)

    obs, info = env.reset()
    interface = info["interface"]
    iter = 0
    flag = False

    while True:
        action = 0 * np.random.randn(3)
        obs, reward, done, trunc, info = env.step(action)
        print(info['reward_components'])
        if done or trunc:
            obs, info = env.reset()
        if flag:
            flag = False
            obs, info = env.reset()
