"""InputDataManager manages the inputs to the NN algorithm.
If replay_file is specified, it will replay the data from the file.
Otherwise, it will connect to the zmq sockets to get the data real-time. 
"""
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
        return self.rgb_streaming_socket.recv()

    def get_stereo_img(self):
        return self.stereo_streaming_socket.recv()
