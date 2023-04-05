from robomimic.utils.file_utils import policy_from_checkpoint
import numpy as np
from scipy.spatial.transform import Rotation as R
from observation_converter import ObservationConverter


class NNWrapper():
    def __init__(self, path):
        self.eval_policy = policy_from_checkpoint(ckpt_path=path)[0]
        self.action_handler = ActionHandler()
        self.obs_converter = ObservationConverter()

    def reset(self, obs):
        """
        Set the initial values of the hand poses based on observation.
        Since the rpc code multiplies the output of the neural network 
        by the zero position and orientation, we need to subtract the zero 
        positions and orientations before sending. We can do this by changing
        the initial pose. 
        """
        obs = self.obs_converter.convert(obs)
        zero_pos = np.array([0.0, 0.0, 0.1])
        zero_quat = np.array([0.0, -.707, 0.0, .707])
        self.action_handler.reset({
            'lh_pos': obs['obs/act_local_lh_pos'] - zero_pos,
            'rh_pos': obs['obs/act_local_rh_pos'] - zero_pos,
            'lh_ori': (R.from_quat(obs['obs/act_local_lh_ori']) * R.from_quat(zero_quat).inv()).as_quat(),
            'rh_ori': (R.from_quat(obs['obs/act_local_rh_ori']) * R.from_quat(zero_quat).inv()).as_quat(),
        })

    def forward(self, obs):
        raw_action = np.array(self.eval_policy(
            self.obs_converter.convert(obs)))
        self.action_handler.update(raw_action)
        return self.action_handler.get_action()


class ActionHandler():
    def __init__(self):

        self._cur_command = {
            'l_gripper': None,
            'r_gripper': None,
            'lh_pos': None,
            'rh_pos': None,
            'lh_ori': None,
            'rh_ori': None,
            'locomotion': None,
        }

    def reset(self, cur_command):
        self._cur_command.update(cur_command)

    def update(self, action):

        logit_locomotion = action[0]
        logit_l_gripper = action[1]
        logit_r_gripper = action[2]

        delta_rh_pos = action[3:6]
        delta_lh_pos = action[6:9]
        delta_rh_ori = action[9:13]
        delta_lh_ori = action[13:17]

        self._cur_command['locomotion'] = logit_locomotion
        self._cur_command['l_gripper'] = np.round(logit_l_gripper)
        self._cur_command['r_gripper'] = np.round(logit_r_gripper)

        self._cur_command['rh_pos'] += delta_rh_pos
        self._cur_command['lh_pos'] += delta_lh_pos

        act_trajecory_right_rot = R.from_quat(
            self._cur_command['rh_ori']).as_matrix() @ R.from_quat(delta_rh_ori).as_matrix()
        act_trajecory_left_rot = R.from_quat(
            self._cur_command['lh_ori']).as_matrix() @ R.from_quat(delta_lh_ori).as_matrix()

        self._cur_command['rh_ori'] = R.from_matrix(
            act_trajecory_right_rot).as_quat()
        self._cur_command['lh_ori'] = R.from_matrix(
            act_trajecory_left_rot).as_quat()

    def get_action(self):
        """
        Returns the current action
        """
        return self._cur_command

    def evaluate(path, **kwargs):

        # CALLED ONLY ONCE AT THE BEGINNING
    eval_policy = policy_from_checkpoint(ckpt_path=path)[0]
    action_handler = ActionHandler()
    obs_handler = ObsHandler()

    raw_obs = env.reset()
    cur_command = None
    action_handler.reset(cur_command)
    obs_handler.reset(raw_obs)

    # THIS IS CALLED EVERY TIME STEP
    while env.cur_time < 20:

        raw_action = np.array(eval_policy(obs))
        action_handler.update(raw_action)
        mapped_action = action_handler.get_action()

        raw_obs, _, _, _ = env.step(mapped_action)
        obs_handler.update(raw_obs)
        obs_handler.get_obs()
