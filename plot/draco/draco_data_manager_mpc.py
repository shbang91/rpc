import zmq
import sys
import os


import ruamel.yaml as yaml
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd + "/build")
sys.path.append(cwd)

from messages.draco_pb2 import *
from plot.data_saver import *

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

import meshcat
import meshcat_shapes
from plot import meshcat_utils as vis_tools

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--b_visualize", type=bool, default=False)
parser.add_argument("--b_use_plotjuggler", type=bool, default=False)
args = parser.parse_args()

##==========================================================================
##Misc
##==========================================================================
des_com_traj_label = []
des_ori_traj_label = []
des_lf_pos_traj_label = []
des_rf_pos_traj_label = []
des_lf_ori_traj_label = []
des_rf_ori_traj_label = []

##==========================================================================
##Socket initialize
##==========================================================================
context = zmq.Context()
socket = context.socket(zmq.SUB)

b_using_kf_estimator = False
b_using_non_kf_estimator = False


def check_if_kf_estimator(kf_pos, est_pos):
    global b_using_kf_estimator, b_using_non_kf_estimator

    # check if we have already set either the KF or non-KF flag to True
    if b_using_kf_estimator or b_using_non_kf_estimator:
        return

    # if both kf_pos and est_pos data are zero's, we have not entered standup
    if not (np.any(kf_pos) or np.any(est_pos)):
        return

    # otherwise, we can infer from the current kf_pos and est_pos data
    if np.any(kf_pos):
        b_using_kf_estimator = True
    else:
        b_using_non_kf_estimator = True


##YAML parse
with open("config/draco/pnc.yaml", "r") as yaml_file:
    try:
        config = yaml.safe_load(yaml_file)
        ip_address = config["ip_address"]
    except yaml.YAMLError as exc:
        print(exc)

socket.connect(ip_address)
socket.setsockopt_string(zmq.SUBSCRIBE, "")

if args.b_use_plotjuggler:
    pj_context = zmq.Context()
    pj_socket = pj_context.socket(zmq.PUB)
    pj_socket.bind("tcp://*:9872")

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
    viz.loadViewerModel(rootNodeName="draco3")
    vis_q = pin.neutral(model)

    # add other visualizations to viewer
    meshcat_shapes.point(
        viz.viewer["com_des"], color=vis_tools.Color.RED, opacity=1.0, radius=0.01
    )

    meshcat_shapes.point(
        viz.viewer["com"], color=vis_tools.Color.BLUE, opacity=1.0, radius=0.01
    )

    meshcat_shapes.point(
        viz.viewer["com_projected"],
        color=vis_tools.Color.BLACK,
        opacity=1.0,
        radius=0.01,
    )

    # add arrows visualizers to viewer
    vis_tools.add_arrow(viz.viewer, "grf_lf", color=[0, 0, 1])  ## BLUE
    vis_tools.add_arrow(viz.viewer, "grf_rf", color=[1, 0, 0])  ## RED

while True:
    ##receive msg trough socket
    encoded_msg = socket.recv()
    msg.ParseFromString(encoded_msg)

    # save data in pkl file
    # data_saver.add('time', msg.time)
    # data_saver.add('phase', msg.phase)

    # save proto msg for python plotting
    # data_saver.advance()

    ## publish back to plot juggler
    # if args.b_use_plotjuggler:
    # pj_socket.send_string(json.dumps(data_saver.history))

    if args.b_visualize:
        check_if_kf_estimator(msg.kf_base_joint_pos, msg.est_base_joint_pos)

        base_pos = (
            msg.kf_base_joint_pos if b_using_kf_estimator else msg.est_base_joint_pos
        )
        base_ori = (
            msg.kf_base_joint_ori if b_using_kf_estimator else msg.est_base_joint_ori
        )

        vis_q[0:3] = np.array(base_pos)
        vis_q[3:7] = np.array(base_ori)  # quaternion [x,y,z,w]
        vis_q[7:] = np.array(msg.joint_positions)

        viz.display(vis_q)

        trans = meshcat.transformations.translation_matrix(
            [msg.des_com_pos[0], msg.des_com_pos[1], msg.des_com_pos[2]]
        )
        viz.viewer["com_des"].set_transform(trans)

        trans = meshcat.transformations.translation_matrix(
            [msg.act_com_pos[0], msg.act_com_pos[1], msg.act_com_pos[2]]
        )
        viz.viewer["com"].set_transform(trans)

        trans = meshcat.transformations.translation_matrix(
            [msg.des_com_pos[0], msg.des_com_pos[1], 0.0]
        )
        viz.viewer["com_projected"].set_transform(trans)

        # visualize GRFs
        if msg.phase != 1:
            vis_tools.grf_display(
                viz.viewer["grf_lf"], msg.lfoot_pos, msg.lfoot_ori, msg.lfoot_rf_cmd
            )
            vis_tools.grf_display(
                viz.viewer["grf_rf"], msg.rfoot_pos, msg.rfoot_ori, msg.rfoot_rf_cmd
            )

        ## visualize des CoM trajectory
        if len(msg.des_com_traj) > 0:
            if len(des_com_traj_label) == 0:
                for i in range(len(msg.des_com_traj)):
                    label = "des_com_" + str(i)
                    des_com_traj_label.append(label)
                for name in des_com_traj_label:
                    meshcat_shapes.point(
                        viz.viewer[name],
                        color=vis_tools.Color.GREEN,
                        opacity=0.8,
                        radius=0.01,
                    )
            else:
                for com, name in zip(msg.des_com_traj, des_com_traj_label):
                    trans = meshcat.transformations.translation_matrix(
                        [com.x, com.y, com.z]
                    )
                    viz.viewer[name].set_transform(trans)

        ## visualize des orientation trajectory
        if len(msg.des_torso_ori_traj) > 0:
            if len(des_ori_traj_label) == 0:
                for i in range(len(msg.des_torso_ori_traj)):
                    label = "des_ori_" + str(i)
                    des_ori_traj_label.append(label)
                for name in des_ori_traj_label:
                    meshcat_shapes.frame(
                        viz.viewer[name],
                        axis_length=0.1,
                        axis_thickness=0.005,
                        opacity=0.8,
                        origin_radius=0.01,
                    )
            else:
                for com, torso_ori, name in zip(
                    msg.des_com_traj, msg.des_torso_ori_traj, des_ori_traj_label
                ):
                    rotation = meshcat.transformations.euler_matrix(
                        torso_ori.x, torso_ori.y, torso_ori.z, axes="sxyz"
                    )
                    trans = meshcat.transformations.translation_matrix(
                        [com.x, com.y, com.z]
                    )
                    viz.viewer[name].set_transform(trans.dot(rotation))

        ## visualize left foot position trajectory
        if len(msg.des_lf_pos_traj) > 0:
            if len(des_lf_pos_traj_label) == 0:
                for i in range(len(msg.des_lf_pos_traj)):
                    label = "des_lf_pos_" + str(i)
                    des_lf_pos_traj_label.append(label)
                for name in des_lf_pos_traj_label:
                    meshcat_shapes.point(
                        viz.viewer[name],
                        color=vis_tools.Color.CYAN,
                        opacity=0.8,
                        radius=0.01,
                    )
            else:
                for lf_pos, name in zip(msg.des_lf_pos_traj, des_lf_pos_traj_label):
                    trans = meshcat.transformations.translation_matrix(
                        [lf_pos.x, lf_pos.y, lf_pos.z]
                    )
                    viz.viewer[name].set_transform(trans)

        ## visualize right foot position trajectory
        if len(msg.des_rf_pos_traj) > 0:
            if len(des_rf_pos_traj_label) == 0:
                for i in range(len(msg.des_rf_pos_traj)):
                    label = "des_rf_pos_" + str(i)
                    des_rf_pos_traj_label.append(label)
                for name in des_rf_pos_traj_label:
                    meshcat_shapes.point(
                        viz.viewer[name],
                        color=vis_tools.Color.YELLOW,
                        opacity=0.8,
                        radius=0.01,
                    )
            else:
                for rf_pos, name in zip(msg.des_rf_pos_traj, des_rf_pos_traj_label):
                    trans = meshcat.transformations.translation_matrix(
                        [rf_pos.x, rf_pos.y, rf_pos.z]
                    )
                    viz.viewer[name].set_transform(trans)

        ## visualize left foot orientation trajectory
        if len(msg.des_lf_ori_traj) > 0:
            if len(des_lf_ori_traj_label) == 0:
                for i in range(len(msg.des_lf_ori_traj)):
                    label = "des_lf_ori_" + str(i)
                    des_lf_ori_traj_label.append(label)
                for name in des_lf_ori_traj_label:
                    meshcat_shapes.frame(
                        viz.viewer[name],
                        axis_length=0.1,
                        axis_thickness=0.005,
                        opacity=0.8,
                        origin_radius=0.01,
                    )
            else:
                for lf_pos, lf_ori, name in zip(
                    msg.des_lf_pos_traj, msg.des_lf_ori_traj, des_lf_ori_traj_label
                ):
                    rotation = meshcat.transformations.euler_matrix(
                        lf_ori.x, lf_ori.y, lf_ori.z, axes="sxyz"
                    )
                    trans = meshcat.transformations.translation_matrix(
                        [lf_pos.x, lf_pos.y, lf_pos.z]
                    )
                    viz.viewer[name].set_transform(trans.dot(rotation))

        ## visualize right foot orientation trajectory
        if len(msg.des_rf_ori_traj) > 0:
            if len(des_rf_ori_traj_label) == 0:
                for i in range(len(msg.des_rf_ori_traj)):
                    label = "des_rf_ori_" + str(i)
                    des_rf_ori_traj_label.append(label)
                for name in des_rf_ori_traj_label:
                    meshcat_shapes.frame(
                        viz.viewer[name],
                        axis_length=0.1,
                        axis_thickness=0.005,
                        opacity=0.8,
                        origin_radius=0.01,
                    )
            else:
                for rf_pos, rf_ori, name in zip(
                    msg.des_rf_pos_traj, msg.des_rf_ori_traj, des_rf_ori_traj_label
                ):
                    rotation = meshcat.transformations.euler_matrix(
                        rf_ori.x, rf_ori.y, rf_ori.z, axes="sxyz"
                    )
                    trans = meshcat.transformations.translation_matrix(
                        [rf_pos.x, rf_pos.y, rf_pos.z]
                    )
                    viz.viewer[name].set_transform(trans.dot(rotation))
