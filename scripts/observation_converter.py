"""Helper class to convert the local/global etc. representations of robot data
"""
from scipy.spatial.transform import Rotation as R
import numpy as np

global_act_trajectory_keys = ["obs/act_global_lh_pos", "obs/act_global_rh_pos", "obs/act_global_lh_ori", "obs/act_global_rh_ori",
                              "obs/act_global_lf_pos", "obs/act_global_rf_pos", "obs/act_global_lf_ori", "obs/act_global_rf_ori"]
# the actions are actually global. TODO: rename them
global_action_keys = ["action/local_rh_pos", "action/local_lh_pos",
                      "action/local_rh_ori", "action/local_lh_ori"]
global_des_trajectory_keys = ["obs/des_global_lh_pos", "obs/des_global_rh_pos", "obs/des_global_lh_ori", "obs/des_global_rh_ori",
                              "obs/des_global_lf_pos", "obs/des_global_rf_pos", "obs/des_global_lf_ori", "obs/des_global_rf_ori"]


class ObservationConverter():
    """
    A class to take observations from either hdf5 or protobuf and converts them to
    a representation used to train or evaluate the neural network.
    Note that the protobuf messages will be singular messages while the hdf5 data 
    will be vectors. The code can deal with both vectors and single messages.
    """

    def __init__(self, include_images=True, normalize_images=False, crop_images=False, include_actions=False, include_desired=False, trim_demo_video=False):
        self.include_actions = include_actions
        self.include_desired = include_desired
        self.trim_demo_video = trim_demo_video 
        self.include_images = include_images
        self.normalize_images = normalize_images
        self.crop_images = crop_images
        self.converted_data = {}

    def convert(self, raw_data):
        """
        Converts the data to the representation used by the neural network.
        Parameters:
            raw_data: the protobuf data from the robot or the hdf5 file from collected demo
        Returns:
            a dictionary with the converted data
        """
        self.get_joint_embedding(
            raw_data['obs/joint_pos'], raw_data['obs/joint_vel'])
        self.get_local_trajectories(raw_data)
        if self.include_images:
            self.get_images(raw_data['obs/rgb'], raw_data['obs/stereo'])
        if self.include_actions:
            self.get_flattened_action_delta(raw_data)
        if self.trim_demo_video:
            self.trim_video_pick(raw_data['action/l_gripper'], raw_data['action/r_gripper'])
        return self.converted_data

    def get_local_trajectories(self, raw_data):
        """
        Gets the [obs/action]/[act/des]_global_[lh/rh/lf/rf]_[pos/ori] from the robot and converts
        them to local. The converted data will be named "local" instead of "global".
        The action is used only for collecting the demonstrations - they are not part of the observation.

        Parameters:
            raw_data: the protobuf data from the robot or the hdf5 file from collected demo
            actions: true if we convert actions
            desired: true if we convert desired values
        Returns:
            a dictionary with the converted data
        """

        # the [()] is to get the numpy array from the h5py dataset. It returns a view of the np array.
        # https://stackoverflow.com/questions/14689270/in-numpy-what-does-indexing-an-array-with-the-empty-tuple-vs-ellipsis-do
        global_base_ori = R.from_quat(
            raw_data['global_base_ori'][()])
        global_base_pos = raw_data['global_base_pos'][()]

        global_params = global_act_trajectory_keys.copy()
        if self.include_actions:
            global_params += global_action_keys
        if self.include_desired:
            global_params += global_des_trajectory_keys

        for param in global_params:
            if param.endswith("pos"):
                self.converted_data[param.replace("global", "local")] = pos_global_to_local(
                    global_base_pos, global_base_ori, raw_data[param][()])
            else:
                self.converted_data[param.replace("global", "local")] = ori_global_to_local(
                    global_base_ori, R.from_quat(raw_data[param][()]))

    def get_images(self, rgb, stereo):
        """
        Gets the rgb and stereo images from the robot. Split the stereo image to left and right.
        Parameters:
            rgb: the rgb image from the robot
            stereo: the stereo image from the robot
        """
        # flatten the images TODO: remove this so it's always flattened
        #stereo = stereo[()].reshape((stereo.shape[0], -1))
        #rgb = rgb[()].reshape((rgb.shape[0], -1))
        # the usage of ... and this reshape trip allows us to process both vector and single images
        # for more information about the reshape call, see
        # https://stackoverflow.com/questions/46183967/how-to-reshape-only-last-dimensions-in-numpy
        #stereo=np.frombuffer(stereo, dtype=np.uint8)
        #rgb=np.frombuffer(rgb, dtype=np.uint8)
        #stereo = stereo.reshape(stereo.shape[:-1] + (200, 800, 1))
        #rgb = rgb.reshape(rgb.shape[:-1] + (200, 400, 3))
        if (self.crop_images):
            rgb = rgb[..., 70:200, :, :]
            stereo = stereo[..., 70:200, :, :]

        self.converted_data['obs/rgb'] = rgb[..., :, :]
        self.converted_data['obs/stereo_0'] = stereo[..., :400, :]
        self.converted_data['obs/stereo_1'] = stereo[..., 400:, :]

        if self.normalize_images:
            self.converted_data['obs/rgb'] = self.converted_data['obs/rgb'].transpose([2, 0, 1])/255.0
            self.converted_data['obs/stereo_0'] = self.converted_data['obs/stereo_0'].transpose([2, 0, 1])/255.0
            self.converted_data['obs/stereo_1'] = self.converted_data['obs/stereo_1'].transpose([2, 0, 1])/255.0
        # uncompress image. Not doing compression right now
        # demo_file['obs/rgb'] = cv2.imdecode(demo_file['obs/rgb'], cv2.IMREAD_COLOR)
        # demo_file['obs/stereo'] = cv2.imdecode(demo_file['obs/stereo'], 0)

        # convert an opencv image to a rgb, rightside-up image
        # obs_group.create_dataset('rgb', data = np.flip(cv2.cvtColor(demo_file['obs/rgb'][()], cv2.COLOR_BGR2RGB), axis=1))
        # obs_group.create_dataset('stereo', data = np.flip(demo_file['obs/stereo'][()], axis=1))

    def trim_video(self, l_gripper, r_gripper):
        # Trim the end of demo videos to be a second after the gripper is last used
        num_images = l_gripper.shape[0] 
        i = num_images - 1
        while l_gripper[i] == 0 and r_gripper[i] == 0 and i > 0:
            i -= 1
        i += 40 # a second is 20 frames
        for key in self.converted_data.keys():
            self.converted_data[key] = self.converted_data[key][:i]

    def trim_video_pick(self, l_gripper, r_gripper):
        # Trim the end of demo videos to be a second after the gripper is last used
        num_images = l_gripper.shape[0] 
        i = 0 
        while i < num_images and l_gripper[i] == 0 and r_gripper[i] == 0:
            i += 1
        i += 60 # a second is 20 frames
        for key in self.converted_data.keys():
            self.converted_data[key] = self.converted_data[key][:i]

    def get_joint_embedding(self, joint_pos, joint_vel):
        """
        cos/sin encoding of joint pos concat with joint vel
        """
        self.converted_data['obs/joint'] = np.concatenate(
            (np.cos(joint_pos), np.sin(joint_pos), joint_vel), axis=len(joint_pos.shape) - 1)

    def get_flattened_action_delta(self, raw_data):
        """
        Computes the delta action for trajectories and flatten them with discrete actions.
        """
        # flatten actions
        act_discrete = np.column_stack(
            [raw_data['action/l_gripper'], raw_data['action/r_gripper']])

        # compute delta pos for action trajectory
        act_trajecory_right_pos = self.converted_data['action/local_rh_pos']
        act_trajecory_left_pos = self.converted_data['action/local_lh_pos']
        act_trajecory_right_quat = self.converted_data['action/local_rh_ori']
        act_trajecory_left_quat = self.converted_data['action/local_lh_ori']

        act_trajecory_right_delta_pos = np.copy(
            act_trajecory_right_pos)
        act_trajecory_right_delta_pos[1:
                                      ] -= act_trajecory_right_pos[: -1]
        act_trajecory_right_delta_pos[0] -= act_trajecory_right_pos[0]
        act_trajecory_left_delta_pos = np.copy(
            act_trajecory_left_pos)
        act_trajecory_left_delta_pos[1:] -= act_trajecory_left_pos[:-1]
        act_trajecory_left_delta_pos[0] -= act_trajecory_left_pos[0]

        act_trajecory_right_rot = R.from_quat(
            act_trajecory_right_quat).as_matrix()
        act_trajecory_right_delta_rot = np.copy(
            act_trajecory_right_rot)
        act_trajecory_right_delta_rot[1:] = act_trajecory_right_delta_rot[1:] @ (
            act_trajecory_right_rot[:-1].transpose(0, 2, 1))
        act_trajecory_right_delta_rot[0] = act_trajecory_right_delta_rot[0] @ (
            act_trajecory_right_rot[0].transpose())
        act_trajecory_right_delta_quat = R.from_matrix(
            act_trajecory_right_delta_rot).as_quat()
        act_trajecory_left_rot = R.from_quat(
            act_trajecory_left_quat).as_matrix()
        act_trajecory_left_delta_rot = np.copy(
            act_trajecory_left_rot)
        act_trajecory_left_delta_rot[1:] = act_trajecory_left_delta_rot[1:] @ (
            act_trajecory_left_rot[:-1].transpose(0, 2, 1))
        act_trajecory_left_delta_rot[0] = act_trajecory_left_delta_rot[0] @ (
            act_trajecory_left_rot[0].transpose())
        act_trajecory_left_delta_quat = R.from_matrix(
            act_trajecory_left_delta_rot).as_quat()
        act_trajecory = np.column_stack(
            [act_trajecory_right_delta_pos, act_trajecory_left_delta_pos, act_trajecory_right_delta_quat, act_trajecory_left_delta_quat])

        act_concat = np.column_stack([act_discrete, act_trajecory])

        act_concat_delay = np.copy(act_concat)
        act_concat_delay[:-1] = act_concat[1:]
        #self.converted_data['actions'] = act_concat
        self.converted_data['actions'] = act_concat_delay

def pos_global_to_local(global_base_pos, global_base_ori, pos):
    return global_base_ori.apply(pos - global_base_pos, inverse=True)


def ori_global_to_local(global_base_ori, ori):
    return (global_base_ori.inv() * ori).as_quat()
