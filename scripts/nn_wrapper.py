from robomimic.utils.file_utils import policy_from_checkpoint
import numpy as np
from scipy.spatial.transform import Rotation as R


class NNWrapper():
    def __init__(self, path):
        self.eval_policy = policy_from_checkpoint(ckpt_path=path)[0]
        self.action_handler = ActionHandler()

    def reset(self, cur_command):
        self.action_handler.reset(cur_command)

    def forward(self, obs):
        raw_action = np.array(self.eval_policy(obs))
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

        self._cur_command['right_pos'] += delta_rh_pos
        self._cur_command['left_pos'] += delta_lh_pos

        act_trajecory_right_rot = R.from_quat(
            self._cur_command['rh_ori']).as_matrix() @ R.from_quat(delta_rh_ori).as_matrix()
        act_trajecory_left_rot = R.from_quat(
            self._cur_command['lh_ori']).as_matrix() @ R.from_quat(delta_lh_ori).as_matrix()

        self._cur_command['rh_ori'] = R.from_matrix(
            act_trajecory_right_rot).as_quat()
        self._cur_command['lh_ori'] = R.from_matrix(
            act_trajecory_left_rot).as_quat()

    def get_action(self):
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
