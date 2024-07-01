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
from copy import copy as COPY

class DracoEnvMpcFreq_Ly_10_dist_video(DracoEnv_v2):
    def __init__(self, mpc_freq, sim_dt, eval = None, burn_in: bool = False, reduced_obs_size: bool = False, render: bool = False, disturbance: bool = False, video = None) -> None:
        super().__init__(mpc_freq=mpc_freq, sim_dt=sim_dt, reduced_obs_size=reduced_obs_size, render=render, eval = eval, disturbance = disturbance, video = video)
        
        #self._reduced_obs_size = reduced_obs_size
        self._burn_in = burn_in
        if mpc_freq == 0:
            print("FREQ SET TO 0. PLEASE INCREASE FREQ")
            #raise Warning
        
        self._set_max_steps_iter(32*150)

        """
        self._freq_push_dict = {'long_push_x': [572, 10, 0], 'short_push_x': [6, 80, 0],
                                'long_push_y': [572, 0, 10], 'short_push_y': [6, 0, 100]}
        """
        """        
        #Trained with this
        self._freq_push_dict = {'long_push_x': [572, 20, 0], 'short_push_x': [10, 350, 0],
                                'long_push_y': [572, 0, 25], 'short_push_y': [10, 0, 350]}
        """
        """
        self._freq_push_dict = {'long_push_x': [572, 30, 0], 'short_push_x': [10, 500, 0],
                                'long_push_y': [572, 0, 20], 'short_push_y': [10, 0, 400]}
        """   
        self._freq_push_dict = {'long_push_x': [572, 20, 0], 'short_push_x': [10, 350, 0],
                                'long_push_y': [572, 0, 20], 'short_push_y': [10, 0, 350],
                                'mid_push_x': [57, 80, 0], 'mid_push_y': [57, 0, 80]}
        
        self._freq_push_selection = {'short_push_x': [10, 1000, 0], 
                                     'mid_push_y': [57, 0, -160], 
                                     'mid_push_x': [57, -175, 0], 
                                     'long_push_y': [572, 0, 30],
                                     'long_push_x': [57, 40, 0], 
                                     'short_push_y': [10, 0, -700]}

        #


        self._push_counter = 0

        self._push_trigger_ini = 2500
        self._push_trigger =  COPY(self._push_trigger_ini)

        self._push_stack = []
        for i in self._freq_push_selection:
            self._push_stack.append(self._freq_push_selection[i])
        print(self._push_stack)

        self._push_ = [-1, -1, -1]
        self._new_step_bool = False
        self._sim_iter_to_mid_swing = 78
        self._counter = -1


        #raise Warning
        if eval is not None:
            with open('test/alip/disturbance.txt', 'w'):
                pass


    def _set_observation_space(self):
        if self._reduced_obs_size:
            self.observation_space = gym.spaces.Box(  #observation space added Tr and previous full_action x and y
                low = np.array([-100]*20),
                high = np.array([100]*20),
                dtype = np.float64
            )
        else:
            self.observation_space = gym.spaces.Box(  #observation space
                low = np.array([-100]*74),
                high = np.array([100]*74),
                dtype = np.float64
            )


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
        COM = np.concatenate((np.array([wbc_obs[0]]), 
                                        wbc_obs[4:10], 
                                        wbc_obs[13:19], 
                                        wbc_obs[24:27],
                                        wbc_obs[19:23]))
        COM[16] -= self._sim_dt 
        if(self._reduced_obs_size):
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
        _wbc_action = 0.1*action
        if self._burn_in:
            _wbc_action = 0*_wbc_action
        return _wbc_action

    def _compute_termination(self, _wbc_obs=None):
        if np.abs(_wbc_obs[23] - 12) < 0.5:  #12 is the alip state
            done = False
            if _wbc_obs is not None:
                #condition = np.any((_wbc_obs[6] < 0.5) | (_wbc_obs[6] > 0.8))  #0.69
                if _wbc_obs[6] > 1.3:
                    if (self._eval is not None):
                        print("high height")
                    self._push_trigger = COPY(self._push_trigger_ini)
                    
                    done = True
                if _wbc_obs[6] < 0.1:
                    if (self._eval is not None):
                        print("low height")
                    self._push_trigger = COPY(self._push_trigger_ini)
                    done = True
                if np.abs(_wbc_obs[7]) > (np.abs(self._Lx_main+_wbc_obs[1])+150):
                    if (self._eval is not None):
                        print("high Lx")
                    self._push_trigger =  COPY(self._push_trigger_ini)
                    #done = True    

                if np.abs(_wbc_obs[8] - _wbc_obs[2]) > 100:
                    if (self._eval is not None):
                        print("high Ly")    
                    self._push_trigger =  COPY(self._push_trigger_ini) 
                    #done = True  
            if (done):
                #CHANGE
                self._freq_push_dict['short_push_x'][1] -= 2*50
                #self._freq_push_dict['short_push_y'][2] -= 2*50
                return True          
        return False
    
    def set_action_command_in_sensor_data(self):
        #maybe set also time in newer version
        dir_command = np.array((0, 10, 0))

        initial_stance_leg = np.random.choice(np.array([-1, 1]))

        self._rpc_draco_sensor_data.initial_stance_leg_ = initial_stance_leg
        self._rpc_draco_sensor_data.policy_command_ = dir_command


    def _set_reward_coeffs(self):
        #reward terms
        self._w_roll_pitch = -1
        self._w_com_height = -1
        self._w_penalise_excessive_Lx = 0.6
        self._w_desired_Lx = 0.4
        self._w_desired_Ly = 2
        self._w_desired_yaw = 1.
        self._w_excessive_fp = -1
        self._w_excessive_angle = -3
        self._w_termination = 0
        self._w_alive_bonus = 0.3
        self._w_intra_pol = 0.04
        self._w_intra_Lx = 0.01
        self._w_intra_Ly = 0.005



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
            if (self._push_trigger <= 0): self._new_step_bool = True
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
                                        self.reward_tracking_yaw(), self.reward_com_height(),
                                        self.reward_roll_pitch(), self.penalise_excessive_fp(),
                                        self.penalise_excessive_yaw()])
        else:
            reward = self.r_intra_different_policy()
            reward += self.r_intra_excessive_Lx()
            reward += self.r_intra_excessive_Ly()
            self.reward_info = np.array([self.r_intra_different_policy(), 
                                         self.r_intra_excessive_Lx(), 
                                         self.r_intra_excessive_Ly()])
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
        
        error = np.square(error/0.1)
        error = np.exp(-error)

        error *= self._w_desired_Lx
        return error

    def penalise_outside_Lx_bounds(self):
        error = np.abs(self._new_wbc_obs[7] - self._old_wbc_obs[1]) - self._Lx_main
        error /= (self._mass*self._zH)

        if error < 0:
            return self._w_penalise_excessive_Lx
        error = np.square(error)
        error = np.exp(-error/0.01)
        return error
        
    
    def reward_tracking_com_Ly(self):
        error = self._old_wbc_obs[2] - self._new_wbc_obs[8]
        error /= (self._mass*self._zH)

        error = np.square(error)
        error = np.exp(-error/0.01)

        error *= self._w_desired_Ly
        return error

    def reward_tracking_yaw(self):
        error = self._new_wbc_obs[15] - self._old_wbc_obs[15] - self._old_wbc_obs[3]
        #error = np.square(error)
        #eror = np.exp(-error)
        error = np.square(error)
        error = np.exp(-error/0.01)
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
        error /= self._sim_dt*self._mpc_freq
        error = np.sum(np.square(error))
        error = np.exp(-error/0.01)
        return self._w_intra_pol * error
    
    def r_intra_excessive_Lx(self):
        error = np.abs(self._new_wbc_obs[7] - self._old_wbc_obs[1]) - self._Lx_main
        error /= (self._mass*self._zH)
        
        if error < 0:
            return self._w_intra_Lx
        
        error = np.exp(-error/0.01)
        return self._w_intra_Lx * error
    
    def r_intra_excessive_Ly(self):
        error = self._old_wbc_obs[2] - self._new_wbc_obs[8]
        error /= (self._mass*self._zH)
        error = np.square(error)
        error = np.exp(-error/0.04)
        return self._w_intra_Ly * error


    def apply_disturbance(self):
        #print("dfa")
        self.apply_disturbance_video()
        """
        print(cwd+'/test/alip/disturbance.txt')
        self._push_trigger -= 1
        #print(self._push_trigger)
        if self._push_trigger == 0:
            timing = np.random.randint(500)
            if timing == 0:
                choice = np.random.randint(0,4)
                if choice == 0:
                    self._push_ = copy.deepcopy(self._freq_push_dict['short_push_x'])
                elif choice == 1:

                    self._push_ = copy.deepcopy(self._freq_push_dict['short_push_y'])
                elif choice == 2:
                    self._push_ = copy.deepcopy(self._freq_push_dict['long_push_x'])
                else :
                    self._push_ = copy.deepcopy(self._freq_push_dict['long_push_y'])

                self._push_dir = np.random.choice([-1, 1])
                if (self._eval is not None):
                    print("choice: ", choice, ", dir: ", self._push_dir) 
            else:
                self._push_trigger = 1 
        if self._push_[0] > 0: 
            self._push_[0] -= 1
            force = np.array((self._push_[1], self._push_[2],0))
            force *= self._push_dir
            #print(force)  
            self.client.applyExternalForce(self.robot, -1, force, np.zeros(3), flags = self.client.LINK_FRAME)
            #print("push")
            if self._push_[0] == 0: self._push_trigger = 2500

        if self._eval is not None:
            if self._push_[0] >= 0:
                force = np.array((self._push_[1], self._push_[2],0))
                with open('test/alip/disturbance.txt', 'a') as file:
                    file.write(f"{force}\n")
            else:
                with open('test/alip/disturbance.txt', 'a') as file:
                    file.write("-1\n")
    """

    def _set_push_trigger(self):
        return COPY(self._push_trigger_ini)
    

    def apply_disturbance_video(self):
        self._push_trigger -= 1
        if (self._push_trigger <= 0) and (self._new_step_bool):
            self._new_step_bool = False
            self._push_trigger = 1000000
            self._counter = 0

        if self._counter >= 0: self._counter += 1
        
        if (self._counter == self._sim_iter_to_mid_swing):
            #CHANGE
            print("hey")
            if self._push_stack:  # Check if list_stack is not empty
                self._push_ = copy.deepcopy(self._push_stack.pop(0))
                self._counter = -1
                #CHANGE
                self._push_dir = 1
            else:
                print("List is empty.")



        if self._push_[0] > 0: 
            print("push", self._push_[0])
            self._push_[0] -= 1
            force = np.array((self._push_[1], self._push_[2],0))
            force *= self._push_dir
            #print(force)  
            self.client.applyExternalForce(self.robot, -1, force, np.zeros(3), flags = self.client.LINK_FRAME)
            #print("push")
            if self._push_[0] == 0: self._push_trigger = COPY(self._push_trigger_ini)
            
        if self._push_[0] >= 0:
            if (self._push_[0] == 0): 
                self._push_[0] = -1
                print(self._push_)

                #CHANGE
                self._freq_push_dict['short_push_x'][1] += 50
                #self._freq_push_dict['short_push_y'][2] += 50
                print("push_end")
            force = np.array((self._push_[1], self._push_[2],0))
            force *= self._push_dir
            with open('test/alip/disturbance.txt', 'a') as file:
                file.write(f"{force}\n")
        else:
            with open('test/alip/disturbance.txt', 'a') as file:
                file.write("-1\n")




    def apply_disturbance_3(self):
        self._push_trigger -= 1
        if (self._push_trigger <= 0) and (self._new_step_bool):
            self._new_step_bool = False
            self._push_trigger = 1000000
            self._counter = 0

        if self._counter >= 0: self._counter += 1
        
        if (self._counter == self._sim_iter_to_mid_swing):
            #CHANGE
            print("hey")
            self._push_ = copy.deepcopy(self._freq_push_dict['short_push_x'])
            self._counter = -1
            #CHANGE
            self._push_dir = 1


        if self._push_[0] > 0: 
            print("push", self._push_[0])
            self._push_[0] -= 1
            force = np.array((self._push_[1], self._push_[2],0))
            force *= self._push_dir
            #print(force)  
            self.client.applyExternalForce(self.robot, -1, force, np.zeros(3), flags = self.client.LINK_FRAME)
            #print("push")
            if self._push_[0] == 0: self._push_trigger = COPY(self._push_trigger_ini)
            
        if self._push_[0] >= 0:
            if (self._push_[0] == 0): 
                self._push_[0] = -1
                print(self._push_)

                #CHANGE
                self._freq_push_dict['short_push_x'][1] += 50
                #self._freq_push_dict['short_push_y'][2] += 50
                print("push_end")
            force = np.array((self._push_[1], self._push_[2],0))
            force *= self._push_dir
            with open('test/alip/disturbance.txt', 'a') as file:
                file.write(f"{force}\n")
        else:
            with open('test/alip/disturbance.txt', 'a') as file:
                file.write("-1\n")

if __name__ == "__main__":
    env = DracoEnvMpcFreq_Ly_10_dist( 5, Config.CONTROLLER_DT, reduced_obs_size=True, render = True)
    from stable_baselines3.common.env_checker import check_env
    check_env(env)

    obs, info = env.reset()
    interface = info["interface"]
    iter = 0
    flag = False

    while True:
        action = 0*np.random.randn(3)
        obs, reward, done, trunc, info = env.step(action)
        print(info['reward_components'])
        if done or trunc:
            obs,info = env.reset()
        if flag:
            flag = False
            obs,info = env.reset()
