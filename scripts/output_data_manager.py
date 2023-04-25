import numpy as np
import os
import sys
import zmq

cwd = os.getcwd()
sys.path.append(cwd + '/build')
sys.path.append(cwd)
from messages.draco_pb2 import *  # noqa


class OutputDataManager:
    """
    Converts the output of the neural network to protobuf and sends to rpc 
    """
    def __init__(self, context, output_port=5555):
        self.socket = context.socket(zmq.PUB)
        self.socket.set(zmq.CONFLATE, 1)
        self.socket.bind("tcp://*:" + str(output_port))
        self.cnt = 0

    def send(self, action):
        """
        Sends the action to the rpc
        """
        msg = vr_teleop_msg()
        if self.cnt < 5:
            msg.l_pad = True
            self.cnt += 1
        if 'l_pad' in action:
            msg.l_pad = action['l_pad']
        msg.lh_pos[:] = action['lh_pos']
        msg.rh_pos[:] = action['rh_pos']
        msg.lh_ori[:] = action['lh_ori']
        msg.rh_ori[:] = action['rh_ori']

        msg.l_bump = action['l_gripper']
        msg.r_bump = action['r_gripper']

        print(action['r_gripper'])

        self.socket.send(msg.SerializeToString())

    def toggle_vr_ready(self):
        self.send({"l_gripper": 0, "r_gripper": 0, "l_pad": True, "lh_pos": np.zeros(3), "rh_pos": np.zeros(3), "lh_ori": np.zeros(9), "rh_ori": np.zeros(9)})
