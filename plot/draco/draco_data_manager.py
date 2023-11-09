import asyncio
import math

import numpy
import time
from base64 import b64encode
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from mcap_protobuf.schema import build_file_descriptor_set

import zmq
import sys
import os

import time

import ruamel.yaml as yaml
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd + '/build')
sys.path.append(cwd)

from util.python_utils.util import rot_to_quat

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
parser.add_argument("--b_use_plotjuggler", type=bool, default=False)
args = parser.parse_args()

##==========================================================================
##Socket initialize
##==========================================================================
context = zmq.Context()
socket = context.socket(zmq.SUB)

b_using_kf_estimator = False
b_using_non_kf_estimator = False

def isMesh(geometry_object):
    """Check whether the geometry object contains a Mesh supported by MeshCat"""
    if geometry_object.meshPath == "":
        return False

    _, file_extension = os.path.splitext(geometry_object.meshPath)
    if file_extension.lower() in [".dae", ".obj", ".stl"]:
        return True

    return False


async def main():
    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        scene_chan_id = await server.add_channel(
            {
                "topic": "scene",
                "encoding": "protobuf",
                "schemaName": SceneUpdate.DESCRIPTOR.full_name,
                "schema": b64encode(
                    build_file_descriptor_set(SceneUpdate).SerializeToString()
                ).decode("ascii"),
            }
        )
        tf_chan_id = await server.add_channel(
            {
                "topic": "transforms",
                "encoding": "protobuf",
                "schemaName": FrameTransform.DESCRIPTOR.full_name,
                "schema": b64encode(
                    build_file_descriptor_set(FrameTransform).SerializeToString()
                ).decode("ascii"),
            }
        )

        class Listener(FoxgloveServerListener):
            def on_subscribe(self, server, channel_id):
                # Send the model data only when a client subscribes, to save bandwidth
                if channel_id == scene_chan_id:
                    scene_update = SceneUpdate()

                asyncio.create_task(
                    server.send_message(
                        scene_chan_id, now, scene_update.SerializeToString()
                    )
                )

            def on_unsubscribe(self, server, channel_id):
                pass

        server.set_listener(Listener())

        # Send the FrameTransform every frame to update the model's position
        transform = FrameTransform()

        while True:
            ##receive msg trough socket
            encoded_msg = socket.recv()
            msg.ParseFromString(encoded_msg)

            ##print pnc msg
            #print(msg)

            #save data in pkl file
            data_saver.add('time', msg.time)
            data_saver.add('phase', msg.phase)

            data_saver.add('est_base_joint_pos', list(msg.est_base_joint_pos))
            data_saver.add('est_base_joint_ori', list(msg.est_base_joint_ori))
            data_saver.add('kf_base_joint_pos', list(msg.kf_base_joint_pos))
            data_saver.add('kf_base_joint_ori', list(msg.kf_base_joint_ori))

            data_saver.add('joint_positions', list(msg.joint_positions))

            data_saver.add('des_com_pos', list(msg.des_com_pos))
            data_saver.add('act_com_pos', list(msg.act_com_pos))

            data_saver.add('lfoot_pos', list(msg.lfoot_pos))
            data_saver.add('rfoot_pos', list(msg.rfoot_pos))

            data_saver.add('lfoot_ori', list(msg.lfoot_ori))
            data_saver.add('rfoot_ori', list(msg.rfoot_ori))

            data_saver.add('lfoot_rf_cmd', list(msg.lfoot_rf_cmd))
            data_saver.add('rfoot_rf_cmd', list(msg.rfoot_rf_cmd))

            data_saver.add('b_lfoot', msg.b_lfoot)
            data_saver.add('b_rfoot', msg.b_rfoot)
            data_saver.add('lfoot_volt_normal_raw', msg.lfoot_volt_normal_raw)
            data_saver.add('rfoot_volt_normal_raw', msg.rfoot_volt_normal_raw)
            data_saver.add('lfoot_rf_normal', msg.lfoot_rf_normal)
            data_saver.add('rfoot_rf_normal', msg.rfoot_rf_normal)
            data_saver.add('lfoot_rf_normal_filt', msg.lfoot_rf_normal_filt)
            data_saver.add('rfoot_rf_normal_filt', msg.rfoot_rf_normal_filt)

            data_saver.add('est_icp', list(msg.est_icp))
            data_saver.add('des_icp', list(msg.des_icp))

            data_saver.add('des_cmp', list(msg.des_cmp))

            # data_saver.add('base_joint_pos', list(msg.base_joint_pos))
            # data_saver.add('base_joint_ori', list(msg.base_joint_ori))
            # data_saver.add('base_joint_lin_vel', list(msg.base_joint_lin_vel))
            # data_saver.add('base_joint_ang_vel', list(msg.base_joint_ang_vel))

            data_saver.add('com_xy_weight', list(msg.com_xy_weight))
            data_saver.add('com_xy_kp', list(msg.com_xy_kp))
            data_saver.add('com_xy_kd', list(msg.com_xy_kd))
            data_saver.add('com_xy_ki', list(msg.com_xy_ki))

            data_saver.add('com_z_weight', msg.com_z_weight)
            data_saver.add('com_z_kp', msg.com_z_kp)
            data_saver.add('com_z_kd', msg.com_z_kd)

            data_saver.add('torso_ori_weight', list(msg.torso_ori_weight))
            data_saver.add('torso_ori_kp', list(msg.torso_ori_kp))
            data_saver.add('torso_ori_kd', list(msg.torso_ori_kd))

            data_saver.add('lf_pos_weight', list(msg.lf_pos_weight))
            data_saver.add('lf_pos_kp', list(msg.lf_pos_kp))
            data_saver.add('lf_pos_kd', list(msg.lf_pos_kd))

            data_saver.add('rf_pos_weight', list(msg.rf_pos_weight))
            data_saver.add('rf_pos_kp', list(msg.rf_pos_kp))
            data_saver.add('rf_pos_kd', list(msg.rf_pos_kd))

            data_saver.add('lf_ori_weight', list(msg.lf_ori_weight))
            data_saver.add('lf_ori_kp', list(msg.lf_ori_kp))
            data_saver.add('lf_ori_kd', list(msg.lf_ori_kd))

            data_saver.add('rf_ori_weight', list(msg.rf_ori_weight))
            data_saver.add('rf_ori_kp', list(msg.rf_ori_kp))
            data_saver.add('rf_ori_kd', list(msg.rf_ori_kd))

            data_saver.add('quat_world_local', list(msg.quat_world_local))

            data_saver.advance()

            #if args.b_use_plotjuggler:
             #   pj_socket.send_string(json.dumps(data_saver.history))

            await asyncio.sleep(0.02)
            now = time.time_ns()
            transform.timestamp.FromNanoseconds(now)

            # Get mesh pose.
            if b_using_kf_estimator:
                base_pos = msg.kf_base_joint_pos
                base_ori = msg.kf_base_joint_ori
            else:
                base_pos = msg.est_base_joint_pos
                base_ori = msg.est_base_joint_ori
            vis_q = pin.neutral(model)
            vis_q[0:3] = np.array(base_pos)
            vis_q[3:7] = np.array(base_ori)  # quaternion [x,y,z,w]
            vis_q[7:] = np.array(msg.joint_positions)
            #MeshcatVisualizer.display(model, q_q)
            #print(q_q)
            #return

            vis_q[0:3] = np.array(base_pos)
            vis_q[3:7] = np.array(base_ori)  # quaternion [x,y,z,w]
            vis_q[7:] = np.array(msg.joint_positions)

            # viz.display(vis_q)

            x, y, z = 0, 0, 0
            pp = "world"

            pin.forwardKinematics(model, data, vis_q)
            pin.updateGeometryPlacements(model, data, visual_model, visual_data)
            for visual in visual_model.geometryObjects:
                # Get mesh pose.
                M = visual_data.oMg[visual_model.getGeometryId(visual.name)]
                # Manage scaling
                if isMesh(visual):
                    scale = np.asarray(visual.meshScale).flatten()
                    S = np.diag(np.concatenate((scale, [1.0])))
                    T = np.array(M.homogeneous).dot(S)
                else:
                    T = M.homogeneous
                anti = visual.name[:-2] #use for frame_id
                transform.parent_frame_id = pp
                # if (visual_model.getGeometryId(visual.name) == 1) or (visual_model.getGeometryId(visual.name) == 9) or (visual_model.getGeometryId(visual.name) == 16) or (visual_model.getGeometryId(visual.name) == 17) or (visual_model.getGeometryId(visual.name) == 25):
                #     transform.parent_frame_id = "torso_link"
                transform.child_frame_id = anti
                x = T[0][3]
                y = T[1][3]
                z = T[2][3]
                transform.translation.x = x
                transform.translation.y = y
                transform.translation.z = z
                rot = T[:3, :3]
                q = rot_to_quat(rot)
                transform.rotation.x = q[0]
                transform.rotation.y = q[1]
                transform.rotation.z = q[2]
                transform.rotation.w = q[3]
                await server.send_message(tf_chan_id, now, transform.SerializeToString())
                asyncio.sleep(10)
                transform.rotation.Clear()
                transform.translation.Clear()


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
        "robot_model/draco/draco_modified.urdf", "robot_model/draco",
        pin.JointModelFreeFlyer())

    data, collision_data, visual_data = pin.createDatas(model, collision_model, visual_model)
    #foxglove testing
    # viz = MeshcatVisualizer(model, collision_model, visual_model)
    # try:
    #     viz.initViewer(open=True)
    # except ImportError as err:
    #     print(
    #         "Error while initializing the viewer. It seems you should install python meshcat"
    #     )
    #     print(err)
    #     exit()
    # viz.loadViewerModel(rootNodeName="draco3")
    vis_q = pin.neutral(model)

    asyncio.run(main())
    exit()

    # viz = MeshcatVisualizer(model, collision_model, visual_model)
    # try:
    #     viz.initViewer(open=True)
    # except ImportError as err:
    #     print(
    #         "Error while initializing the viewer. It seems you should install python meshcat"
    #     )
    #     print(err)
    #     exit()
    # viz.loadViewerModel(rootNodeName="draco3")
    # vis_q = pin.neutral(model)

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
    vis_tools.add_arrow(arrow_viz, "grf_lf_normal", color=[0.2, 0.2, 0.2, 0.2])
    vis_tools.add_arrow(arrow_viz, "grf_rf_normal", color=[0.2, 0.2, 0.2, 0.2])

while True:
    ##----------------------------------------------##
    ##all this stuff was moved into foxglove publisher
    ##----------------------------------------------##

    ## publish back to plot juggler
    if args.b_use_plotjuggler:
        pj_socket.send_string(json.dumps(data_saver.history))

    if args.b_visualize:
        check_if_kf_estimator(msg.kf_base_joint_pos, msg.est_base_joint_pos)

        if b_using_kf_estimator:
            base_pos = msg.kf_base_joint_pos
            base_ori = msg.kf_base_joint_ori
        else:
            base_pos = msg.est_base_joint_pos
            base_ori = msg.est_base_joint_ori

        vis_q[0:3] = np.array(base_pos)
        vis_q[3:7] = np.array(base_ori)  # quaternion [x,y,z,w]
        vis_q[7:] = np.array(msg.joint_positions)

        com_des_viz_q[0] = msg.des_com_pos[0]
        com_des_viz_q[1] = msg.des_com_pos[1]
        com_des_viz_q[2] = msg.des_com_pos[2]

        com_viz_q[0] = msg.act_com_pos[0]
        com_viz_q[1] = msg.act_com_pos[1]
        com_viz_q[2] = msg.act_com_pos[2]

        com_proj_viz_q[0] = msg.des_com_pos[0]
        com_proj_viz_q[1] = msg.des_com_pos[1]

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
        com_des_viz.display(com_des_viz_q)
        com_viz.display(com_viz_q)
        com_proj_viz.display(com_proj_viz_q)
        icp_viz.display(icp_viz_q)
        icp_des_viz.display(icp_des_viz_q)
        cmp_des_viz.display(cmp_des_viz_q)

        # plot GRFs
        if msg.phase != 1:
            vis_tools.grf_display(arrow_viz["grf_lf"], msg.lfoot_pos,
                                  msg.lfoot_ori, msg.lfoot_rf_cmd)
            vis_tools.grf_display(arrow_viz["grf_rf"], msg.rfoot_pos,
                                  msg.rfoot_ori, msg.rfoot_rf_cmd)

            # add sensed normal force
            lfoot_rf_normal = np.array([0., 0., 0., 0., 0., msg.lfoot_rf_normal_filt])
            rfoot_rf_normal = np.array([0., 0., 0., 0., 0., msg.rfoot_rf_normal_filt])
            vis_tools.grf_display(arrow_viz["grf_lf_normal"], msg.lfoot_pos,
                                  msg.lfoot_ori, lfoot_rf_normal)
            vis_tools.grf_display(arrow_viz["grf_rf_normal"], msg.rfoot_pos,
                                  msg.rfoot_ori, rfoot_rf_normal)

