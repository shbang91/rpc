"""InputDataManager manages the inputs to the NN algorithm.
If replay_file is specified, it will replay the data from the file.
Otherwise, it will connect to the zmq sockets to get the data real-time. 
"""
from robomimic.utils.file_utils import h5py
import zmq
import sys
import os
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd + '/build')
sys.path.append(cwd)
from messages.draco_pb2 import *  # noqa


class InputDataManager:
    def get_robot_data(self):
        """Gets the robot data from the zmq socket or replay file.
        Returns:
            The robot data in this format:
            ├── action
            │   ├── l_gripper 
            │   ├── local_lh_ori 
            │   ├── local_lh_pos 
            │   ├── local_rh_ori 
            │   ├── local_rh_pos 
            │   └── r_gripper 
            ├── est_base_joint_ori 
            ├── est_base_joint_pos 
            ├── kf_base_joint_ori 
            ├── kf_base_joint_pos 
            ├── obs
            │   ├── joint_pos 
            │   ├── joint_vel 
            │   ├── local_lf_ori 
            │   ├── local_lf_pos 
            │   ├── local_lh_ori 
            │   ├── local_lh_pos 
            │   ├── local_rf_ori 
            │   ├── local_rf_pos 
            │   ├── local_rh_ori 
            │   ├── local_rh_pos 
            │   └── state 
            └── timestamp 
        """
        raise NotImplementedError

    def get_rgb_img(self):
        """Gets the rgb image from the zmq socket or replay file.
        Returns:
            The rgb image.
        """
        raise NotImplementedError

    def get_stereo_img(self):
        """Gets the stereo image from the zmq socket or replay file.
        Returns:
            The stereo image.
        """
        raise NotImplementedError


class SocketInputDataManager(InputDataManager):
    def __init__(self, context, control_ip, rgb_camera_ip, stereo_camera_ip):
        """Initializes the socket input data manager.
        Args:
            context: The zmq context.
            control_ip: The ip of the control pc.
            rgb_camera_ip: The ip of the rgb camera.
            stereo_camera_ip: The ip of the stereo camera.
        """
        self.control_socket = context.socket(zmq.SUB)

        self.control_socket.connect(control_ip)
        self.control_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.control_socket.set(zmq.CONFLATE, 1)

        self.rgb_streaming_socket = context.socket(zmq.PULL)
        self.rgb_streaming_socket.set(zmq.CONFLATE, 1)
        self.rgb_streaming_socket.connect(rgb_camera_ip)

        self.stereo_streaming_socket = context.socket(zmq.PULL)
        self.stereo_streaming_socket.set(zmq.CONFLATE, 1)
        self.stereo_streaming_socket.connect(stereo_camera_ip)

    def get_robot_data(self):
        msg = pnc_msg()
        msg.ParseFromString(self.control_socket.recv())
        return msg

    def get_rgb_img(self):
        #return np.zeros(200 * 400 * 3)
        return np.frombuffer(self.rgb_streaming_socket.recv(), dtype=np.uint8).reshape(200, 400, 3)

    def get_stereo_img(self):
        #return np.zeros(200 * 800)
        return np.frombuffer(self.stereo_streaming_socket.recv(), dtype=np.uint8).reshape(200, 800, 1)

class ReplayInputDataManager(InputDataManager):
    def __init__(self, replay_file):
        """Initializes the replay input data manager.
        Args:
            replay_file: The replay file.
        """
        self.file = h5py.File(replay_file, "r")
        self.replay_index = 0

    def get_robot_data(self):
        if self.replay_index >= len(self.file['obs/rgb']):
            return None
        msg = pnc_msg()
        msg.act_global_lh_pos[:] = self.file['obs/act_global_lh_pos'][self.replay_index]
        msg.act_global_lh_ori[:] = self.file['obs/act_global_lh_ori'][self.replay_index]
        msg.act_global_rh_pos[:] = self.file['obs/act_global_rh_pos'][self.replay_index]
        msg.act_global_rh_ori[:] = self.file['obs/act_global_rh_ori'][self.replay_index]
        msg.act_global_lf_pos[:] = self.file['obs/act_global_lf_pos'][self.replay_index]
        msg.act_global_lf_ori[:] = self.file['obs/act_global_lf_ori'][self.replay_index]
        msg.act_global_rf_pos[:] = self.file['obs/act_global_rf_pos'][self.replay_index]
        msg.act_global_rf_ori[:] = self.file['obs/act_global_rf_ori'][self.replay_index]

        msg.global_base_pos[:] = self.file['global_base_pos'][self.replay_index]
        msg.global_base_ori[:] = self.file['global_base_ori'][self.replay_index]

        msg.joint_positions[:] = self.file['obs/joint_pos'][self.replay_index]
        msg.joint_velocities[:] = self.file['obs/joint_vel'][self.replay_index]

        self.replay_index += 1
        return msg

    def get_rgb_img(self):
        return self.file['obs/rgb'][self.replay_index]

    def get_stereo_img(self):
        return self.file['obs/stereo'][self.replay_index]
