import zmq
import sys
import os

import time

import ruamel.yaml as yaml
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd + '/build')
sys.path.append(cwd)

from messages.draco_pb2 import *
from plot.data_saver import *

# import pinocchio as pin
# from pinocchio.visualize import MeshcatVisualizer

import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--b_visualize", type=bool, default=False)
args = parser.parse_args()

##==========================================================================
##Socket initialize
##==========================================================================
context = zmq.Context()
socket = context.socket(zmq.SUB)

##YAML parse
with open("config/draco/pnc.yaml", "r") as yaml_file:
    try:
        config = yaml.safe_load(yaml_file)
        ip_address = config["ip_address"]
    except yaml.YAMLError as exc:
        print(exc)

socket.connect(ip_address)
socket.setsockopt_string(zmq.SUBSCRIBE, "")

pj_context = zmq.Context()
pj_socket = pj_context.socket(zmq.PUB)
pj_socket.bind("tcp://*:9872")

msg = pnc_msg()

data_saver = DataSaver()

##meshcat visualizer
# if args.b_visualize:
# model, collision_model, visual_model = pin.buildModelsFromUrdf(
# "robot_model/draco/draco.urdf", "robot_model/draco",
# pin.JointModelFreeFlyer())
# viz = MeshcatVisualizer(model, collision_model, visual_model)
# try:
# viz.initViewer(open=True)
# except ImportError as err:
# print(
# "Error while initializing the viewer. It seems you should install python meshcat"
# )
# print(err)
# exit()
# viz.loadViewerModel()
# vis_q = pin.neutral(model)

while True:
    ##receive msg trough socket
    encoded_msg = socket.recv()
    msg.ParseFromString(encoded_msg)

    ##print pnc msg
    # print(msg)

    #save data in pkl file
    data_saver.add('time', msg.time)

    data_saver.add('base_joint_pos', list(msg.base_joint_pos))
    data_saver.add('base_joint_ori', list(msg.base_joint_ori))
    data_saver.add('base_joint_lin_vel', list(msg.base_joint_lin_vel))
    data_saver.add('base_joint_ang_vel', list(msg.base_joint_ang_vel))

    data_saver.add('est_base_joint_pos', list(msg.est_base_joint_pos))
    data_saver.add('est_base_joint_ori', list(msg.est_base_joint_ori))
    data_saver.add('est_base_joint_lin_vel', list(msg.est_base_joint_lin_vel))
    data_saver.add('est_base_joint_ang_vel', list(msg.est_base_joint_ang_vel))

    data_saver.add('des_com_pos', list(msg.des_com_pos))
    data_saver.add('act_com_pos', list(msg.act_com_pos))
    data_saver.add('des_com_vel', list(msg.des_com_vel))
    data_saver.add('act_com_vel', list(msg.act_com_vel))

    ##TODO: TEST

    # fb_msg = msg.fb
    # print(fb_msg.bjoint_pos)
    ##TODO: TEST

    data_saver.advance()

    ## publish back to plot juggler
    pj_socket.send_string(json.dumps(data_saver.history))

    if args.b_visualize:
        vis_q[0:3] = np.array(msg.base_joint_pos)
        vis_q[3:7] = np.array(msg.base_joint_ori)  # quaternion [x,y,z,w]
        vis_q[7:] = np.array(msg.joint_positions)

        viz.display(vis_q)
