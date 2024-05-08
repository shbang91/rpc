import os
import sys


cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/build')
from config.draco.pybullet_simulation import *
import zmq
import time
import ruamel.yaml as yaml
import numpy as np
from util.python_utils.util import rot_to_quat
from messages.draco_pb2 import *
from plot.data_saver import *
import pinocchio as pin
import json
import argparse
from scipy.spatial.transform import Rotation as R

if Config.USE_FOXGLOVE:
    import webbrowser
    import UI.foxglove.control_widgets as foxglove_ctrl
    from foxglove_websocket.types import Parameter
    import asyncio
    import threading
    # load parameters that can be controlled / changed
    param_store = foxglove_ctrl.load_params_store()
    step_listener = foxglove_ctrl.Listener(param_store)

parser = argparse.ArgumentParser()
parser.add_argument("--b_use_plotjuggler", type=bool, default=False)
parser.add_argument("--visualizer", choices=['none', 'meshcat', 'foxglove'], default='none')
args = parser.parse_args()

if args.visualizer == 'meshcat':
    from pinocchio.visualize import MeshcatVisualizer
    import meshcat
    from plot import meshcat_utils as vis_tools
elif args.visualizer == 'foxglove':
    import asyncio
    import math
    import numpy
    from base64 import b64encode
    # Foxglove dependencies
    from foxglove_schemas_protobuf.SpherePrimitive_pb2 import SpherePrimitive
    from foxglove_websocket import run_cancellable
    from foxglove_websocket.server import FoxgloveServer
    from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
    from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
    from mcap_protobuf.schema import build_file_descriptor_set
    # local tools to manage Foxglove scenes
    from plot.foxglove_utils import FoxgloveShapeListener, SceneChannel, ScalableArrowsScene

    scene_schema = b64encode(build_file_descriptor_set(SceneUpdate).SerializeToString()).decode("ascii")
    frame_schema = b64encode(build_file_descriptor_set(FrameTransform).SerializeToString()).decode("ascii")

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
    async with (FoxgloveServer("0.0.0.0", 8765, "Visualization server",capabilities=["parameters", "parametersSubscribe"]) as server):
        tf_chan_id = await SceneChannel(False,"transforms", "protobuf", FrameTransform.DESCRIPTOR.full_name, frame_schema).add_chan(server)
        normS_chan_id = await SceneChannel(False,"normal_viz", "protobuf", SceneUpdate.DESCRIPTOR.full_name, scene_schema).add_chan(server)
        norm_chan_id = await SceneChannel(True,"normal", "json", "normal", ["lfoot_rf_cmd","rfoot_rf_cmd","lfoot_rf_normal_filt","rfoot_rf_normal_filt"]).add_chan(server)
        icpS_chan_id = await SceneChannel(False,"icp_viz", "protobuf", SceneUpdate.DESCRIPTOR.full_name, scene_schema).add_chan(server)
        icp_chan_id = await SceneChannel(True,"icp", "json", "icp", ["est_x","est_y","des_x","des_y"]).add_chan(server)
        steptest = await SceneChannel(True,"steptest", "json", "steptest", ["num_steps"]).add_chan(server)

        #create all of the visual scenes
        scenes = []
        norm_listener = FoxgloveShapeListener(normS_chan_id,"arrows",["lfoot_rf_cmd","rfoot_rf_cmd","lfoot_rf_normal_filt","rfoot_rf_normal_filt"],{"lfoot_rf_cmd":[.1,.1,.1,.2],"rfoot_rf_cmd":[.1,.1,.1,.2],"lfoot_rf_normal_filt":[.1,.1,.1,.2],"rfoot_rf_normal_filt":[.1,.1,.1,.2]},{"lfoot_rf_cmd":[1,0,1,1],"rfoot_rf_cmd":[1,0,1,1],"lfoot_rf_normal_filt":[1,0,1,1],"rfoot_rf_normal_filt":[1,0,1,1]})
        scenes.append(norm_listener)
        icp_listener = FoxgloveShapeListener(icpS_chan_id, "spheres", ["est_icp","des_icp"], {"est_icp":[.1,.1,.1],"des_icp":[.1,.1,.1]}, {"est_icp":[1,0,1,1],"des_icp":[0,1,0,1]})
        scenes.append(icp_listener)
        scenes.append(step_listener)
        arrows_scene = ScalableArrowsScene()
        arrows_scene.add_arrow("lfoot_rf_cmd", [0, 0, 1, 0.5])                   # blue arrow
        arrows_scene.add_arrow("rfoot_rf_cmd", [0, 0, 1, 0.5])                   # blue arrow
        arrows_scene.add_arrow("lfoot_rf_normal_filt", [0.2, 0.2, 0.2, 0.5])     # grey arrow
        arrows_scene.add_arrow("rfoot_rf_normal_filt", [0.2, 0.2, 0.2, 0.5])     # grey arrow
        scenecount = len(scenes)-1

        # Send the FrameTransform every frame to update the model's position
        transform = FrameTransform()

        print("foxglove websocket initiated")

        param_store = step_listener._param_store

        while True:
            #cycle through all visual scenes    --CAUSES A BUG WHERE ALL BUT ONE SCENE NEED TO BE TOGGLED OFF AND BACK ON
            server.set_listener(scenes[scenecount])
            #if the scene is the steplistener, we update the step parameter
            if(scenecount == (len(scenes)-1)): server.update_parameters([Parameter(name="n_steps", value=param_store["n_steps"], type=None)])

            if step_listener.has_been_modified():
                if step_listener.is_cmd_triggered('n_steps'):
                    new_steps_num = step_listener.get_val('n_steps')
                    rpc_draco_interface.interrupt_.PressStepNum(new_steps_num)
                    step_listener.reset()
                    print("exec update frro")

            scenecount = scenecount-1
            if(scenecount == -1): scenecount = (len(scenes)-1)

            # receive msg trough socket
            encoded_msg = socket.recv()
            msg.ParseFromString(encoded_msg)
            check_if_kf_estimator(msg.kf_base_joint_pos, msg.est_base_joint_pos)

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

            #send 2 pairs of icp x & y as topics to foxglove
            await server.send_message(icp_chan_id, now, json.dumps(
                {"est_x": list(msg.est_icp)[0], "est_y": list(msg.est_icp)[1], "des_x": list(msg.des_icp)[0],
                 "des_y": list(msg.des_icp)[1]}).encode("utf8"))

            #send 2 pairs of l & r norm data as topics to foxglove
            await server.send_message(norm_chan_id, now, json.dumps(
                {"lfoot_rf_cmd": list(msg.lfoot_rf_cmd)[0], "rfoot_rf_cmd": list(msg.rfoot_rf_cmd)[0],
                 "lfoot_rf_normal_filt": msg.lfoot_rf_normal_filt, "rfoot_rf_normal_filt": msg.rfoot_rf_normal_filt}).encode("utf8"))

            #send step data
            await server.send_message(steptest, now, json.dumps(
                {"num_steps": 6}).encode("utf8"))
            #print(steptest[0])


            # update mesh positions
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
                anti = visual.name[:-2]  # use for frame_id
                transform.parent_frame_id = pp
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
                transform.rotation.Clear()
                transform.translation.Clear()

            # update icp values on the grid
            for obj in ["est_icp", "des_icp"]:
                transform.parent_frame_id = "world"
                transform.child_frame_id = obj
                transform.timestamp.FromNanoseconds(now)
                transform.translation.x = list(getattr(msg, obj))[0]
                transform.translation.y = list(getattr(msg, obj))[1]
                await server.send_message(tf_chan_id, now, transform.SerializeToString())

            Ry = R.from_euler('y', -np.pi / 2).as_matrix()
            # update GRF arrows
            for obj in ["lfoot_rf_cmd", "rfoot_rf_cmd", "lfoot_rf_normal_filt", "rfoot_rf_normal_filt"]:
                transform.parent_frame_id = "world"
                transform.child_frame_id = obj
                transform.timestamp.FromNanoseconds(now)
                # show aligned at the center of respective foot sole
                if obj in ["lfoot_rf_cmd", "lfoot_rf_normal_filt"]:
                    R_foot = R.from_quat(msg.lfoot_ori).as_matrix()
                    transform.translation.x = msg.lfoot_pos[0]
                    transform.translation.y = msg.lfoot_pos[1]
                else:
                    R_foot = R.from_quat(msg.rfoot_ori).as_matrix()
                    transform.translation.x = msg.rfoot_pos[0]
                    transform.translation.y = msg.rfoot_pos[1]

                # rotate arrow since it points in +x direction
                R_arrow = R_foot @ Ry
                q_arrow = rot_to_quat(R_arrow)
                transform.rotation.x = q_arrow[0]
                transform.rotation.y = q_arrow[1]
                transform.rotation.z = q_arrow[2]
                transform.rotation.w = q_arrow[3]

                if obj in ["lfoot_rf_cmd", "rfoot_rf_cmd"]:
                    force_dir = np.array(list(getattr(msg, obj))[3:])
                    force_dir /= 1200.        # scale down
                    arrows_scene.update(obj, force_dir, now)
                # else:
                #     norm_listener.size_dict[obj] = [2,.1,.1,.2]
                await server.send_message(tf_chan_id, now, transform.SerializeToString())
                await server.send_message(normS_chan_id, now, arrows_scene.serialized_msg(obj))


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

#
# Visualizer Settings
#
if args.visualizer != 'none':
    # both meshcat and foxglove make use of Pinocchio model and model data
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        "robot_model/draco/draco_modified.urdf", "robot_model/draco",
        pin.JointModelFreeFlyer())

    data, collision_data, visual_data = pin.createDatas(model, collision_model, visual_model)
    vis_q = pin.neutral(model)

    # define and initialize elements to visualize
    if args.visualizer == 'meshcat':
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


def process_data_saver(visualize_type):

    if visualize_type == 'meshcat':
        # save data in pkl file (saved to replay data)
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
        data_saver.add('quat_world_local', list(msg.quat_world_local))

    elif visualize_type == 'foxglove':
        pass

    elif visualize_type == 'none':

        # save data in pkl file (typically, for Plotjuggler)
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


while True:
    #print("\nFLAG_B1")
    # receive msg through socket
    encoded_msg = socket.recv()
    msg.ParseFromString(encoded_msg)
    #print("FLAG_MSG")
    # if publishing raw messages, floating base estimates names are not important
    if args.visualizer != 'none':
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

        if args.visualizer == 'meshcat':
            process_data_saver('meshcat')

            # update visualizer viewers
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
        elif args.visualizer == 'foxglove':
            print("FLAG_FOXTRIG")
            webbrowser.open('https://app.foxglove.dev/speedway/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765')
            y = threading.Thread(target=asyncio.run(main()), args=())
            y.start()
            #asyncio.run(main())

    else:   # if 'none' specified
        process_data_saver('none')

    # publish back to plot juggler
    # note: currently, this is not reached when using foxglove but the corresponding
    # ROS messages can be visualized within foxglove
    if args.b_use_plotjuggler:
        process_data_saver('none')
        pj_socket.send_string(json.dumps(data_saver.history))
