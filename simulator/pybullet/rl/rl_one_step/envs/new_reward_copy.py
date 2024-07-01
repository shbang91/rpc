import gymnasium as gym
import numpy as np
import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")
from util.python_utils import pybullet_util_rl
from config.draco.pybullet_simulation import *

from simulator.pybullet.rl.env import *


class DracoEnvOneStepMpc(DracoEnv):
    def __init__(self, Lx_offset_des, Ly_des, yaw_des, mpc_freq, sim_dt, randomized_command: bool = False, reduced_obs_size: bool = False, render: bool = False) -> None:
        super().__init__(Lx_offset_des, Ly_des, yaw_des, mpc_freq, sim_dt, randomized_command, reduced_obs_size, render)

        if mpc_freq != 0:
            print("FREQ != 0. PLEASE SET FREQ == 0")
            raise Warning

        assert randomized_command == False

        self._set_max_steps_iter(30)
    
    def _set_observation_space(self):
        if self._reduced_obs_size:
            self.observation_space = gym.spaces.Box(  #observation space added Tr and previous full_action x and y
                low = np.array([-100]*13),
                high = np.array([100]*13),
                dtype = np.float64
            )
        else:
            self.observation_space = gym.spaces.Box(  #observation space
                low = np.array([-100]*70),
                high = np.array([100]*70),
                dtype = np.float64
            )

    def _get_observation(self, wbc_obs) -> dict:
        """ Desired state is not an input to nn implicit in the env
        stance_leg
        com_pos_desired_frame x3
        L_desired_frame       x3
        torso_roll_pitch_yaw  x3
        swfoot_roll_pitch_yaw x3
        torso_ang_vel         x3 -- removed
        """
        COM = np.concatenate((np.array([wbc_obs[0]]), wbc_obs[4:10], wbc_obs[13:19]))
        if self._reduced_obs_size:
            policy_obs = COM
        else:
            imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, b_rf_contact, \
                l_normal_force, r_normal_force = pybullet_util_rl.get_sensor_data_from_pybullet(
                self.robot, DracoLinkIdx, DracoJointIdx, self.previous_torso_velocity,
                self.link_id_dict, self.client)
            joint_obs = np.concatenate((joint_pos, joint_vel))
            policy_obs = np.concatenate((joint_obs, COM))

        return policy_obs

    def _normalise_action(self, action):
        _wbc_action = 0.15*action
        return _wbc_action


    def _compute_termination(self, _wbc_obs=None):
        if np.abs(_wbc_obs[23] - 12) < 0.5:  #12 is the alip state
            if _wbc_obs is not None:
                #condition = np.any((_wbc_obs[6] < 0.5) | (_wbc_obs[6] > 0.8))  #0.69
                if _wbc_obs[6] > 1:
                    return True
                if _wbc_obs[6] < 0.45:
                    return True
                if np.abs(_wbc_obs[7]) > (np.abs(self._Lx_main+_wbc_obs[1])+100):
                    return True
                if np.abs(_wbc_obs[8] - _wbc_obs[2]) > 50:
                    return True
        return False

    def _set_reward_coeffs(self):
        #reward terms
        self._w_roll_pitch = -0.5
        self._w_com_height = -1
        self._w_penalise_excessive_Lx = -0.5 
        self._w_desired_Lx = -3.
        self._w_desired_Ly = -3.
        self._w_desired_yaw = -3.
        self._w_excessive_fp = -0.5
        self._w_excessive_angle = -0.5
        self._w_termination = -10.
        self._w_alive_bonus = 5.


    def _compute_reward(self, wbc_obs, action, done):
        if (done): 
            self.reward_info = self._w_termination
            #return self._w_termination/self._iter
            return self._w_termination
        if wbc_obs is None: return 0

        self._old_wbc_obs = np.copy(self._new_wbc_obs)
        self._new_wbc_obs = np.copy(wbc_obs)
        self._rl_action = np.copy(action)
        
        reward = self._w_alive_bonus
        reward += self.reward_tracking_com_Lx()
        reward += self.penalise_outside_Lx_bounds()
        reward += self.reward_tracking_com_Ly()
        reward += self.reward_tracking_yaw()
        reward += self.reward_com_height()
        reward += self.reward_roll_pitch()
        reward += self.penalise_excessive_fp()
        reward += self.penalise_excessive_yaw()
        #if done: reward -= self._w_termination
        self.reward_info = np.array([reward, self._w_alive_bonus,  self.reward_tracking_com_Lx(),
                                    self.penalise_outside_Lx_bounds(), self.reward_tracking_com_Ly(),
                                    999999, self.reward_tracking_yaw(), self.reward_com_height(),
                                    self.reward_roll_pitch(), self.penalise_excessive_fp(),
                                    self.penalise_excessive_yaw()])
        return reward.item()

    def reward_tracking_com_Lx(self):
        if (self._new_wbc_obs[0] == 1):
            L = self._old_wbc_obs[1]+self._Lx_main   #Lx_offset+ LX_MAIN
        else:
            L = self._old_wbc_obs[1]-self._Lx_main
        #in the code 1 corresponds to current stance foot right
        # -1 to current stance foot left
        # new obs -1 --> ended policy for left foot --> we are at the desired state for end of right stance
        error = L - self._new_wbc_obs[7]  #+ self._old_wbc_obs[1:3] - self._new_wbc_obs[9:11]  #desired Lx,y - observedLx,y at the end of the step
        error /= (self._mass*self._zH)
        
        error = np.square(error)
        #error = np.exp(-error)

        error *= self._w_desired_Lx
        return error

    def penalise_outside_Lx_bounds(self):
        error = np.abs(self._new_wbc_obs[7] - self._old_wbc_obs[1]) - self._Lx_main
        error /= (self._mass*self._zH)

        if error > 0:
            error *= self._w_penalise_excessive_Lx
            return error
        return 0
        
    
    def reward_tracking_com_Ly(self):
        error = self._old_wbc_obs[2] - self._new_wbc_obs[8]
        error /= (self._mass*self._zH)

        error = np.square(error)
        #error = np.exp(-error)

        error *= self._w_desired_Ly
        return error

    def reward_tracking_yaw(self):
        error = self._new_wbc_obs[15] - self._old_wbc_obs[15] - self._old_wbc_obs[3]
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

if __name__ == "__main__":
    import math
    yaw = 20
    env = DracoEnvOneStepMpc(0., 0., yaw, 0, Config.CONTROLLER_DT, randomized_command=False, reduced_obs_size=False, render = True)
    #from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    obs, info = env.reset()
    interface = info["interface"]
    iter = 0
    flag = False

    while True:
        action = np.zeros(3)
        obs, reward, done, trunc, info = env.step(action)
        #print(info['reward_components'])
        if done or trunc:
            obs,info = env.reset()
        if flag:
            flag = False
            obs,info = env.reset()
