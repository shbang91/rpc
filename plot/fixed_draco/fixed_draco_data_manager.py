import zmq
import sys
import os
import ruamel.yaml as yaml
import numpy as np
import pinocchio as pin
import argparse

from messages.fixed_draco_pb2 import *
from plot.data_saver import *

from pinocchio.visualize import MeshcatVisualizer

cwd = os.getcwd()
sys.path.append(cwd + "/build")
sys.path.append(cwd)

parser = argparse.ArgumentParser()
parser.add_argument("--b_visualize", type=bool, default=False)
args = parser.parse_args()

context = zmq.Context()
socket = context.socket(zmq.SUB)

##YAML parse
with open("config/fixed_draco/pnc.yaml", "r") as yaml_file:
    try:
        config = yaml.safe_load(yaml_file)
        ip_address = config["ip_address"]
    except yaml.YAMLError as exc:
        print(exc)

socket.connect(ip_address)
socket.setsockopt_string(zmq.SUBSCRIBE, "")

msg = pnc_msg()

data_saver = DataSaver()

##meshcat visualizer
if args.b_visualize:
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        "robot_model/draco/draco_modified.urdf",
        "robot_model/draco",
        pin.JointModelFreeFlyer(),
    )
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    try:
        viz.initViewer(open=True)
    except ImportError as err:
        print(
            "Error while initializing the viewer. It seems you should install python meshcat"
        )
        print(err)
        exit()
    viz.loadViewerModel()
    vis_q = pin.neutral(model)

while True:
    ##receive msg trough socket
    encoded_msg = socket.recv()
    msg.ParseFromString(encoded_msg)

    ##print pnc msg
    # print(msg)

    ##save data in pkl file
    data_saver.add("time", msg.time)
    data_saver.add("base_com_pos", list(msg.base_com_pos))
    data_saver.add("base_com_ori", list(msg.base_com_ori))
    data_saver.add("base_com_lin_vel", list(msg.base_com_lin_vel))
    data_saver.add("base_com_ang_vel", list(msg.base_com_ang_vel))

    data_saver.add("base_joint_pos", list(msg.base_joint_pos))
    data_saver.add("base_joint_ori", list(msg.base_joint_ori))
    data_saver.add("base_joint_lin_vel", list(msg.base_joint_lin_vel))
    data_saver.add("base_joint_ang_vel", list(msg.base_joint_ang_vel))

    data_saver.advance()

    if args.b_visualize:
        vis_q[0:3] = np.array(msg.base_joint_pos)
        vis_q[3:7] = np.array(msg.base_joint_ori)  # quaternion [x,y,z,w]
        vis_q[7:] = np.array(msg.joint_positions)

        viz.display(vis_q)
