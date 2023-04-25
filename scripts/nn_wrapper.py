from robomimic.utils.file_utils import policy_from_checkpoint
import numpy as np
from scipy.spatial.transform import Rotation as R
from observation_converter import ObservationConverter
import h5py

class ReplayRawAction():
    """
    directly replay the local hand poses instead of the post-processed delta poses
    """
    def __init__(self, path):
        self.data = h5py.File(path, "r")
        self.idx = 0

    def reset(self, _):
        pass

    def forward(self, _):
        action = {}

class ReplayAction():
    """
    Instead of generating the actions from the neural network, replay it from a file 
    """
    def __init__(self, path, demo_num = 50):
        self.data = h5py.File(path, "r")["data/demo_" + str(demo_num)]
        self.idx = 0
        self.action_handler = ActionHandler()

    def reset(self, obs):
        converter = ObservationConverter() # not using self.obs_converter since we don't need images
        obs = converter.convert(obs)
        zero_pos = np.array([0.0, 0.0, 0.1])
        zero_quat = np.array([0.0, -.707, 0.0, .707])
        self.action_handler.reset({
            'lh_pos': obs['obs/act_local_lh_pos'] - zero_pos,
            'rh_pos': obs['obs/act_local_rh_pos'] - zero_pos,
            'lh_ori': (R.from_quat(obs['obs/act_local_lh_ori']) * R.from_quat(zero_quat).inv()).as_quat(),
            'rh_ori': (R.from_quat(obs['obs/act_local_rh_ori']) * R.from_quat(zero_quat).inv()).as_quat(),
        })

    # use the same interface as NNWrapper hence the useless argument
    def forward(self, _):
        self.action_handler.update(self.data["actions"][self.idx])
        self.idx += 1
        return self.action_handler.get_action()

    def replay_local_action(self):
        zero_pos = np.array([0.0, 0.0, 0.1])
        zero_quat = np.array([0.0, -.707, 0.0, .707])
        action = {} 
        action['lh_pos'] = self.data["action/local_lh_pos"][self.idx] - zero_pos
        action['rh_pos'] = self.data["action/local_rh_pos"][self.idx] - zero_pos
        action['lh_ori'] = (R.from_quat(self.data["action/local_lh_ori"][self.idx]) * R.from_quat(zero_quat).inv()).as_matrix().flatten()
        action['rh_ori'] = (R.from_quat(self.data["action/local_rh_ori"][self.idx]) * R.from_quat(zero_quat).inv()).as_matrix().flatten()
        action['l_gripper'] = False
        action['r_gripper'] = False
        self.idx += 1
        return action


class NNWrapper():
    def __init__(self, path):
        self.eval_policy = policy_from_checkpoint(ckpt_path=path)[0]
        self.action_handler = ActionHandler()
        self.obs_converter = ObservationConverter(normalize_images=True)

    def reset(self, obs):
        """
        Set the initial values of the hand poses based on observation.
        Since the rpc code multiplies the output of the neural network 
        by the zero position and orientation, we need to subtract the zero 
        positions and orientations before sending. We can do this by changing
        the initial pose. 
        """
        converter = ObservationConverter() # not using self.obs_converter since we don't need images
        obs = converter.convert(obs)
        zero_pos = np.array([0.0, 0.0, 0.1])
        zero_quat = np.array([0.0, -.707, 0.0, .707])
        self.action_handler.reset({
            'lh_pos': obs['obs/act_local_lh_pos'] - zero_pos,
            'rh_pos': obs['obs/act_local_rh_pos'] - zero_pos,
            'lh_ori': (R.from_quat(obs['obs/act_local_lh_ori']) * R.from_quat(zero_quat).inv()).as_quat(),
            'rh_ori': (R.from_quat(obs['obs/act_local_rh_ori']) * R.from_quat(zero_quat).inv()).as_quat(),
        })

    def forward(self, obs):
        obs = self.obs_converter.convert(obs)
        # change the keys to remove the "obs/"
        flat_obs = {}
        for key in obs.keys():
            flat_obs[key.split("/")[1]] = obs[key]
        #print(flat_obs['rgb'].shape)
        raw_action = np.array(self.eval_policy(
flat_obs))
        self.action_handler.update(raw_action)
        #print(self.action_handler.get_action())
        return self.action_handler.get_action()


class ActionHandler():
    def __init__(self):

        self._cur_command = {
            'l_gripper': 0,
            'r_gripper': 0,
            'lh_pos': None,
            'rh_pos': None,
            'lh_ori': None,
            'rh_ori': None,
            'locomotion': None,
        }

    def reset(self, cur_command):
        self._cur_command.update(cur_command)
        self._cur_command['lh_ori'] = R.from_quat(self._cur_command['lh_ori']).as_matrix().flatten()
        self._cur_command['rh_ori'] = R.from_quat(self._cur_command['rh_ori']).as_matrix().flatten()

    def update(self, action):

        logit_l_gripper = action[0]
        logit_r_gripper = action[1]

        delta_rh_pos = action[2:5]
        delta_lh_pos = action[5:8]
        delta_rh_ori = action[8:12]
        delta_lh_ori = action[12:16]

        self._cur_command['l_gripper'] = np.round(logit_l_gripper)
        self._cur_command['r_gripper'] = np.round(logit_r_gripper)

        self._cur_command['rh_pos'] += delta_rh_pos
        self._cur_command['lh_pos'] += delta_lh_pos

        act_trajecory_right_rot = self._cur_command['rh_ori'].reshape((3,3)) @ R.from_quat(delta_rh_ori).as_matrix()
        act_trajecory_left_rot = self._cur_command['lh_ori'].reshape((3,3)) @ R.from_quat(delta_lh_ori).as_matrix()

        self._cur_command['rh_ori'] = act_trajecory_right_rot.flatten()
        self._cur_command['lh_ori'] = act_trajecory_left_rot.flatten()

    def get_action(self):
        """
        Returns the current action
        """
        return self._cur_command
