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

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

import meshcat
from plot import meshcat_utils as vis_tools

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

# pj_context = zmq.Context()
# pj_socket = pj_context.socket(zmq.PUB)
# pj_socket.bind("tcp://*:9872")

msg = pnc_msg()

data_saver = DataSaver()

##meshcat visualizer
if args.b_visualize:
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        "robot_model/draco/draco3_big_feet.urdf", "robot_model/draco",
        pin.JointModelFreeFlyer())
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    try:
        viz.initViewer(open=True)
    except ImportError as err:
        print(
            "Error while initializing the viewer. It seems you should install python meshcat"
        )
        print(err)
        exit()
    viz.loadViewerModel(rootNodeName="draco3")
    vis_q = pin.neutral(model)

    # add other visualizations to viewer
    com_des_viz, com_des_model = vis_tools.add_sphere(viz.viewer,
                                                      "com_des",
                                                      color=[0., 0., 1., 0.5])
    com_des_viz_q = pin.neutral(com_des_model)

    com_viz, com_model = vis_tools.add_sphere(viz.viewer,
                                              "com",
                                              color=[1., 0., 0., 0.5])
    com_viz_q = pin.neutral(com_model)

    com_proj_viz, com_proj_model = vis_tools.add_sphere(
        viz.viewer, "com_proj", color=[0., 0., 1., 0.3])
    com_proj_viz_q = pin.neutral(com_proj_model)

    icp_viz, icp_model = vis_tools.add_sphere(viz.viewer,
                                              "icp",
                                              color=vis_tools.violet)
    icp_viz_q = pin.neutral(icp_model)

    icp_des_viz, icp_des_model = vis_tools.add_sphere(viz.viewer,
                                                      "icp_des",
                                                      color=[0., 1., 0., 0.3])
    icp_des_viz_q = pin.neutral(icp_des_model)

    cmp_des_viz, cmp_des_model = vis_tools.add_sphere(
        viz.viewer, "cmp_des", color=[0., 0.75, 0.75, 0.3])
    cmp_des_viz_q = pin.neutral(cmp_des_model)

    # add arrows visualizers to viewer
    arrow_viz = meshcat.Visualizer(window=viz.viewer.window)
    vis_tools.add_arrow(arrow_viz, "grf_lf", color=[0, 0, 1])
    vis_tools.add_arrow(arrow_viz, "grf_rf", color=[1, 0, 0])

while True:
    ##receive msg trough socket
    encoded_msg = socket.recv()
    msg.ParseFromString(encoded_msg)

    ##print pnc msg
    # print(msg)

    #save data in pkl file
    data_saver.add('time', msg.time)
    data_saver.add('phase', msg.phase)

    data_saver.add('est_base_joint_pos', list(msg.est_base_joint_pos))
    data_saver.add('est_base_joint_ori', list(msg.est_base_joint_ori))

    data_saver.add('joint_positions', list(msg.joint_positions))

    data_saver.add('des_com_pos', list(msg.des_com_pos))
    data_saver.add('act_com_pos', list(msg.act_com_pos))

    data_saver.add('lfoot_pos', list(msg.lfoot_pos))
    data_saver.add('rfoot_pos', list(msg.rfoot_pos))

    data_saver.add('lfoot_ori', list(msg.lfoot_ori))
    data_saver.add('rfoot_ori', list(msg.rfoot_ori))

    data_saver.add('lfoot_rf_cmd', list(msg.lfoot_rf_cmd))
    data_saver.add('rfoot_rf_cmd', list(msg.rfoot_rf_cmd))

    data_saver.add('est_icp', list(msg.est_icp))
    data_saver.add('des_icp', list(msg.des_icp))

    data_saver.add('des_cmp', list(msg.des_cmp))
    # data_saver.add('base_joint_pos', list(msg.base_joint_pos))
    # data_saver.add('base_joint_ori', list(msg.base_joint_ori))
    # data_saver.add('base_joint_lin_vel', list(msg.base_joint_lin_vel))
    # data_saver.add('base_joint_ang_vel', list(msg.base_joint_ang_vel))

    ##TODO: TEST

    # fb_msg = msg.fb
    # print(fb_msg.bjoint_pos)
    ##TODO: TEST

    data_saver.advance()

    ## publish back to plot juggler
    # pj_socket.send_string(json.dumps(data_saver.history))

    if args.b_visualize:
        vis_q[0:3] = np.array(msg.est_base_joint_pos)
        vis_q[3:7] = np.array(msg.est_base_joint_ori)  # quaternion [x,y,z,w]
        vis_q[7:] = np.array(msg.joint_positions)

        icp_viz_q[0] = msg.est_icp[0]
        icp_viz_q[1] = msg.est_icp[1]
        icp_viz_q[2] = 0.

        icp_des_viz_q[0] = msg.des_icp[0]
        icp_des_viz_q[1] = msg.des_icp[1]
        icp_des_viz_q[2] = 0.

        cmp_des_viz_q[0] = msg.des_cmp[0]
        cmp_des_viz_q[1] = msg.des_cmp[1]
        cmp_des_viz_q[2] = 0.

        viz.display(vis_q)
        icp_viz.display(icp_viz_q)
        icp_des_viz.display(icp_des_viz_q)
        cmp_des_viz.display(cmp_des_viz_q)

        # plot GRFs
        vis_tools.grf_display(arrow_viz["grf_lf"], msg.lfoot_pos,
                              msg.lfoot_ori, msg.lfoot_rf_cmd)
        vis_tools.grf_display(arrow_viz["grf_rf"], msg.rfoot_pos,
                              msg.rfoot_ori, msg.rfoot_rf_cmd)
