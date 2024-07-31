import os
import sys
import zmq
import time
import ruamel.yaml as yaml
import numpy as np
from util.python_utils.util import rot_to_quat, quat_to_rot
from messages.draco_pb2 import *
from plot.data_saver import *
import pinocchio as pin
import json
import argparse
from scipy.spatial.transform import Rotation as R

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build")

parser = argparse.ArgumentParser()
parser.add_argument("--b_use_plotjuggler", type=bool, default=False)
parser.add_argument(
    "--visualizer", choices=["none", "meshcat", "foxglove"], default="none"
)
args = parser.parse_args()

if args.visualizer == "meshcat":
    from pinocchio.visualize import MeshcatVisualizer
    import meshcat
    from plot import meshcat_utils as vis_tools
elif args.visualizer == "foxglove":
    # Foxglove dependencies
    import UI.foxglove.control_widgets as foxglove_ctrl
    import asyncio
    import threading
    from base64 import b64encode
    from foxglove_websocket.server import FoxgloveServer
    from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
    from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
    from mcap_protobuf.schema import build_file_descriptor_set

    # local tools to manage Foxglove scenes
    from plot.foxglove_utils import (
        FoxgloveShapeListener,
        SceneChannel,
        ScalableArrowsScene,
    )

    # load parameters that can be controlled / changed and start Control Parameters server
    param_store = foxglove_ctrl.load_params_store()
    step_listener = foxglove_ctrl.Listener(param_store)
    th_slow = threading.Thread(
        target=asyncio.run, args=([foxglove_ctrl.run(step_listener)])
    )
    th_slow.start()

    scene_schema = b64encode(
        build_file_descriptor_set(SceneUpdate).SerializeToString()
    ).decode("ascii")
    frame_schema = b64encode(
        build_file_descriptor_set(FrameTransform).SerializeToString()
    ).decode("ascii")

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


xyz_scene_names = [
    "torso_ori_weight",
    "lf_pos_weight",
    "lf_pos_kp",
    "lf_pos_kd",
    "lf_ori_weight",
    "rf_pos_weight",
    "rf_ori_weight",
]
xyz_scenes = []


async def sceneinitman(name, server):
    x = await SceneChannel(True, name, "json", name, ["x", "y", "z"]).add_chan(server)
    xyz_scenes.append([x, name])
    return x


async def main():
    async with FoxgloveServer(
        "0.0.0.0",
        8765,
        "Visualization server",
        capabilities=["parameters", "parametersSubscribe"],
    ) as server:
        tf_chan_id = await SceneChannel(
            False,
            "transforms",
            "protobuf",
            FrameTransform.DESCRIPTOR.full_name,
            frame_schema,
        ).add_chan(server)
        normS_chan_id = await SceneChannel(
            False,
            "normal_viz",
            "protobuf",
            SceneUpdate.DESCRIPTOR.full_name,
            scene_schema,
        ).add_chan(server)
        grfs_chan_id = await SceneChannel(
            True,
            "GRFs",
            "json",
            "normal",
            [
                "lfoot_rf_cmd_x",
                "rfoot_rf_cmd_x",
                "lfoot_rf_cmd_y",
                "rfoot_rf_cmd_y",
                "lfoot_rf_cmd_z",
                "rfoot_rf_cmd_z",
                "lfoot_rf_normal_filt",
                "rfoot_rf_normal_filt",
            ],
        ).add_chan(server)
        icpS_chan_id = await SceneChannel(
            False, "icp_viz", "protobuf", SceneUpdate.DESCRIPTOR.full_name, scene_schema
        ).add_chan(server)
        icp_chan_id = await SceneChannel(
            True, "icp", "json", "icp", ["est_x", "est_y", "des_x", "des_y"]
        ).add_chan(server)
        for scn in range(len(xyz_scene_names)):
            await sceneinitman(xyz_scene_names[scn], server)

        # create all of the visual scenes
        scenes = []
        norm_listener = FoxgloveShapeListener(
            normS_chan_id,
            "arrows",
            [
                "lfoot_rf_cmd",
                "rfoot_rf_cmd",
                "lfoot_rf_normal_filt",
                "rfoot_rf_normal_filt",
            ],
            {
                "lfoot_rf_cmd": [1, 0.05, 0.05, 0.1],
                "rfoot_rf_cmd": [1, 0.05, 0.05, 0.1],
                "lfoot_rf_normal_filt": [0.1, 0.1, 0.1, 0.2],
                "rfoot_rf_normal_filt": [0.1, 0.1, 0.1, 0.2],
            },
            {
                "lfoot_rf_cmd": [1, 0, 1, 1],
                "rfoot_rf_cmd": [1, 0, 1, 1],
                "lfoot_rf_normal_filt": [1, 0, 1, 1],
                "rfoot_rf_normal_filt": [1, 0, 1, 1],
            },
        )
        scenes.append(norm_listener)
        icp_listener = FoxgloveShapeListener(
            icpS_chan_id,
            "spheres",
            ["est_icp", "des_icp"],
            {"est_icp": [0.1, 0.1, 0.1], "des_icp": [0.1, 0.1, 0.1]},
            {"est_icp": [1, 0, 1, 1], "des_icp": [0, 1, 0, 1]},
        )
        scenes.append(icp_listener)
        arrows_scene = ScalableArrowsScene()
        arrows_scene.add_arrow("lfoot_rf_cmd", [0, 0, 1, 0.5])  # blue arrow
        arrows_scene.add_arrow("rfoot_rf_cmd", [0, 0, 1, 0.5])  # blue arrow
        arrows_scene.add_arrow(
            "lfoot_rf_normal_filt", [0.2, 0.2, 0.2, 0.5]
        )  # grey arrow
        arrows_scene.add_arrow(
            "rfoot_rf_normal_filt", [0.2, 0.2, 0.2, 0.5]
        )  # grey arrow
        scenecount = len(scenes) - 1

        # Send the FrameTransform every frame to update the model's position
        transform = FrameTransform()

        print("foxglove websocket initiated")
        r_foot, l_foot = [0, 0, 0], [0, 0, 0]
        while True:
            # cycle through all visual scenes    --CAUSES A BUG WHERE ALL BUT ONE SCENE NEED TO BE TOGGLED OFF AND BACK ON
            server.set_listener(scenes[scenecount])

            scenecount = scenecount - 1
            if scenecount == -1:
                scenecount = len(scenes) - 1

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

            # send 2 pairs of icp x & y as topics to foxglove
            await server.send_message(
                icp_chan_id,
                now,
                json.dumps(
                    {
                        "est_x": list(msg.est_icp)[0],
                        "est_y": list(msg.est_icp)[1],
                        "des_x": list(msg.des_icp)[0],
                        "des_y": list(msg.des_icp)[1],
                    }
                ).encode("utf8"),
            )

            for scn in xyz_scenes:
                await server.send_message(
                    scn[0],
                    now,
                    json.dumps(
                        {
                            "x": list(getattr(msg, scn[1]))[0],
                            "y": list(getattr(msg, scn[1]))[1],
                            "z": list(getattr(msg, scn[1]))[2],
                        }
                    ).encode("utf8"),
                )

            # send 2 pairs of l & r norm data as topics to foxglove
            await server.send_message(
                grfs_chan_id,
                now,
                json.dumps(
                    {
                        "lfoot_rf_cmd_x": list(msg.lfoot_rf_cmd)[3],
                        "rfoot_rf_cmd_x": list(msg.rfoot_rf_cmd)[3],
                        "lfoot_rf_cmd_y": list(msg.lfoot_rf_cmd)[4],
                        "rfoot_rf_cmd_y": list(msg.rfoot_rf_cmd)[4],
                        "lfoot_rf_cmd_z": list(msg.lfoot_rf_cmd)[5],
                        "rfoot_rf_cmd_z": list(msg.rfoot_rf_cmd)[5],
                        "lfoot_rf_normal_filt": msg.lfoot_rf_normal_filt,
                        "rfoot_rf_normal_filt": msg.rfoot_rf_normal_filt,
                    }
                ).encode("utf8"),
            )

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
                if visual.name == "l_ankle_ie_link_0":
                    l_foot[0] = x
                    l_foot[1] = y
                    l_foot[2] = z
                if visual.name == "r_ankle_ie_link_0":
                    r_foot[0] = x
                    r_foot[1] = y
                    r_foot[2] = z
                transform.translation.x = x
                transform.translation.y = y
                transform.translation.z = z
                rot = T[:3, :3]
                q = rot_to_quat(rot)
                transform.rotation.x = q[0]
                transform.rotation.y = q[1]
                transform.rotation.z = q[2]
                transform.rotation.w = q[3]
                await server.send_message(
                    tf_chan_id, now, transform.SerializeToString()
                )
                transform.rotation.Clear()
                transform.translation.Clear()

            # update icp values on the grid
            for obj in ["est_icp", "des_icp"]:
                transform.parent_frame_id = "world"
                transform.child_frame_id = obj
                transform.timestamp.FromNanoseconds(now)
                transform.translation.x = list(getattr(msg, obj))[0]
                transform.translation.y = list(getattr(msg, obj))[1]
                await server.send_message(
                    tf_chan_id, now, transform.SerializeToString()
                )

            Ry = R.from_euler("y", -np.pi / 2).as_matrix()
            # update GRF arrows
            for obj in [
                "lfoot_rf_cmd",
                "rfoot_rf_cmd",
                "lfoot_rf_normal_filt",
                "rfoot_rf_normal_filt",
            ]:
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

                # rotate transform since arrow points in +x direction
                q_cmd_arrow = rot_to_quat(Ry)
                transform.rotation.x = q_cmd_arrow[0]
                transform.rotation.y = q_cmd_arrow[1]
                transform.rotation.z = q_cmd_arrow[2]
                transform.rotation.w = q_cmd_arrow[3]

                if obj in ["lfoot_rf_cmd", "rfoot_rf_cmd"]:
                    force_dir = np.array(list(getattr(msg, obj))[3:])
                    force_norm = np.linalg.norm(force_dir)

                    # compute axis and angle of rotation to align z with force direction
                    force_dir /= force_norm
                    rot_ang = np.arccos(force_dir.dot(np.array([0, 0, 1])))
                    rot_ax = np.cross(force_dir, np.array([0, 0, 1]))
                    rot_ax /= np.linalg.norm(rot_ax)
                    ax_hat = np.array(
                        [
                            [0, -rot_ax[2], rot_ax[1]],
                            [rot_ax[2], 0, -rot_ax[0]],
                            [-rot_ax[1], rot_ax[0], 0],
                        ]
                    )
                    R_rot_force = (
                        np.eye(3)
                        + np.sin(rot_ang) * ax_hat
                        + (1 - np.cos(rot_ang)) * ax_hat @ ax_hat
                    )
                    quat_force = rot_to_quat(R_rot_force)

                    # force scale
                    force_magnitude = force_norm / 1200.0
                else:
                    force_magnitude = getattr(msg, obj)
                    R_foot_arrow_up_local = R_foot @ Ry
                    q_cmd_arrow = rot_to_quat(R_foot_arrow_up_local)
                    transform.rotation.x = q_cmd_arrow[0]
                    transform.rotation.y = q_cmd_arrow[1]
                    transform.rotation.z = q_cmd_arrow[2]
                    transform.rotation.w = q_cmd_arrow[3]
                    quat_force = np.array(
                        [0, 0, 0, 1]
                    )  # we can only measure Fz for now

                    # force scale
                    force_magnitude = force_magnitude / 1200.0
                arrows_scene.update(obj, quat_force, force_magnitude, now)
                await server.send_message(
                    tf_chan_id, now, transform.SerializeToString()
                )
                await server.send_message(
                    normS_chan_id, now, arrows_scene.serialized_msg(obj)
                )


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
if args.visualizer != "none":
    # both meshcat and foxglove make use of Pinocchio model and model data
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        "robot_model/draco/draco_modified.urdf",
        "robot_model/draco",
        pin.JointModelFreeFlyer(),
    )

    data, collision_data, visual_data = pin.createDatas(
        model, collision_model, visual_model
    )
    vis_q = pin.neutral(model)

    # define and initialize elements to visualize
    if args.visualizer == "meshcat":
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
        com_des_viz, com_des_model = vis_tools.add_sphere(
            viz.viewer, "com_des", color=[0.0, 0.0, 1.0, 0.5]
        )
        com_des_viz_q = pin.neutral(com_des_model)

        com_viz, com_model = vis_tools.add_sphere(
            viz.viewer, "com", color=[1.0, 0.0, 0.0, 0.5]
        )
        com_viz_q = pin.neutral(com_model)

        com_proj_viz, com_proj_model = vis_tools.add_sphere(
            viz.viewer, "com_proj", color=[0.0, 0.0, 1.0, 0.3]
        )
        com_proj_viz_q = pin.neutral(com_proj_model)

        icp_viz, icp_model = vis_tools.add_sphere(
            viz.viewer, "icp", color=vis_tools.violet
        )
        icp_viz_q = pin.neutral(icp_model)

        icp_des_viz, icp_des_model = vis_tools.add_sphere(
            viz.viewer, "icp_des", color=[0.0, 1.0, 0.0, 0.3]
        )
        icp_des_viz_q = pin.neutral(icp_des_model)

        cmp_des_viz, cmp_des_model = vis_tools.add_sphere(
            viz.viewer, "cmp_des", color=[0.0, 0.75, 0.75, 0.3]
        )
        cmp_des_viz_q = pin.neutral(cmp_des_model)

        # add arrows visualizers to viewer
        arrow_viz = meshcat.Visualizer(window=viz.viewer.window)
        vis_tools.add_arrow(arrow_viz, "grf_lf", color=[0, 0, 1])
        vis_tools.add_arrow(arrow_viz, "grf_rf", color=[1, 0, 0])
        vis_tools.add_arrow(arrow_viz, "grf_lf_normal", color=[0.2, 0.2, 0.2, 0.2])
        vis_tools.add_arrow(arrow_viz, "grf_rf_normal", color=[0.2, 0.2, 0.2, 0.2])


def process_data_saver(visualize_type):
    if visualize_type == "meshcat":
        # save data in pkl file (saved to replay data)
        data_saver.add("time", msg.time)
        data_saver.add("phase", msg.phase)
        data_saver.add("est_base_joint_pos", list(msg.est_base_joint_pos))
        data_saver.add("est_base_joint_ori", list(msg.est_base_joint_ori))
        data_saver.add("kf_base_joint_pos", list(msg.kf_base_joint_pos))
        data_saver.add("kf_base_joint_ori", list(msg.kf_base_joint_ori))
        data_saver.add("joint_positions", list(msg.joint_positions))
        data_saver.add("des_com_pos", list(msg.des_com_pos))
        data_saver.add("act_com_pos", list(msg.act_com_pos))
        data_saver.add("lfoot_pos", list(msg.lfoot_pos))
        data_saver.add("rfoot_pos", list(msg.rfoot_pos))
        data_saver.add("lfoot_ori", list(msg.lfoot_ori))
        data_saver.add("rfoot_ori", list(msg.rfoot_ori))
        data_saver.add("lfoot_rf_cmd", list(msg.lfoot_rf_cmd))
        data_saver.add("rfoot_rf_cmd", list(msg.rfoot_rf_cmd))
        data_saver.add("b_lfoot", msg.b_lfoot)
        data_saver.add("b_rfoot", msg.b_rfoot)
        data_saver.add("lfoot_volt_normal_raw", msg.lfoot_volt_normal_raw)
        data_saver.add("rfoot_volt_normal_raw", msg.rfoot_volt_normal_raw)
        data_saver.add("lfoot_rf_normal", msg.lfoot_rf_normal)
        data_saver.add("rfoot_rf_normal", msg.rfoot_rf_normal)
        data_saver.add("lfoot_rf_normal_filt", msg.lfoot_rf_normal_filt)
        data_saver.add("rfoot_rf_normal_filt", msg.rfoot_rf_normal_filt)
        data_saver.add("est_icp", list(msg.est_icp))
        data_saver.add("des_icp", list(msg.des_icp))
        data_saver.add("des_cmp", list(msg.des_cmp))
        data_saver.add("quat_world_local", list(msg.quat_world_local))

    elif visualize_type == "foxglove":
        pass

    elif visualize_type == "none":
        # save data in pkl file (typically, for Plotjuggler)
        data_saver.add("time", msg.time)
        data_saver.add("phase", msg.phase)
        data_saver.add("est_base_joint_pos", list(msg.est_base_joint_pos))
        data_saver.add("est_base_joint_ori", list(msg.est_base_joint_ori))
        data_saver.add("kf_base_joint_pos", list(msg.kf_base_joint_pos))
        data_saver.add("kf_base_joint_ori", list(msg.kf_base_joint_ori))
        data_saver.add("joint_positions", list(msg.joint_positions))
        data_saver.add("des_com_pos", list(msg.des_com_pos))
        data_saver.add("act_com_pos", list(msg.act_com_pos))
        data_saver.add("lfoot_pos", list(msg.lfoot_pos))
        data_saver.add("rfoot_pos", list(msg.rfoot_pos))
        data_saver.add("lfoot_ori", list(msg.lfoot_ori))
        data_saver.add("rfoot_ori", list(msg.rfoot_ori))
        data_saver.add("lfoot_rf_cmd", list(msg.lfoot_rf_cmd))
        data_saver.add("rfoot_rf_cmd", list(msg.rfoot_rf_cmd))
        data_saver.add("b_lfoot", msg.b_lfoot)
        data_saver.add("b_rfoot", msg.b_rfoot)
        data_saver.add("lfoot_volt_normal_raw", msg.lfoot_volt_normal_raw)
        data_saver.add("rfoot_volt_normal_raw", msg.rfoot_volt_normal_raw)
        data_saver.add("lfoot_rf_normal", msg.lfoot_rf_normal)
        data_saver.add("rfoot_rf_normal", msg.rfoot_rf_normal)
        data_saver.add("lfoot_rf_normal_filt", msg.lfoot_rf_normal_filt)
        data_saver.add("rfoot_rf_normal_filt", msg.rfoot_rf_normal_filt)
        data_saver.add("est_icp", list(msg.est_icp))
        data_saver.add("des_icp", list(msg.des_icp))
        data_saver.add("des_cmp", list(msg.des_cmp))
        data_saver.add("com_xy_weight", list(msg.com_xy_weight))
        data_saver.add("com_xy_kp", list(msg.com_xy_kp))
        data_saver.add("com_xy_kd", list(msg.com_xy_kd))
        data_saver.add("com_xy_ki", list(msg.com_xy_ki))
        data_saver.add("com_z_weight", msg.com_z_weight)
        data_saver.add("com_z_kp", msg.com_z_kp)
        data_saver.add("com_z_kd", msg.com_z_kd)
        data_saver.add("torso_ori_weight", list(msg.torso_ori_weight))
        data_saver.add("torso_ori_kp", list(msg.torso_ori_kp))
        data_saver.add("torso_ori_kd", list(msg.torso_ori_kd))
        data_saver.add("lf_pos_weight", list(msg.lf_pos_weight))
        data_saver.add("lf_pos_kp", list(msg.lf_pos_kp))
        data_saver.add("lf_pos_kd", list(msg.lf_pos_kd))
        data_saver.add("rf_pos_weight", list(msg.rf_pos_weight))
        data_saver.add("rf_pos_kp", list(msg.rf_pos_kp))
        data_saver.add("rf_pos_kd", list(msg.rf_pos_kd))
        data_saver.add("lf_ori_weight", list(msg.lf_ori_weight))
        data_saver.add("lf_ori_kp", list(msg.lf_ori_kp))
        data_saver.add("lf_ori_kd", list(msg.lf_ori_kd))
        data_saver.add("rf_ori_weight", list(msg.rf_ori_weight))
        data_saver.add("rf_ori_kp", list(msg.rf_ori_kp))
        data_saver.add("rf_ori_kd", list(msg.rf_ori_kd))
        data_saver.add("quat_world_local", list(msg.quat_world_local))

    data_saver.advance()


while True:
    # print("\nFLAG_B1")
    # receive msg through socket
    encoded_msg = socket.recv()
    msg.ParseFromString(encoded_msg)
    # print("FLAG_MSG")
    # if publishing raw messages, floating base estimates names are not important
    if args.visualizer != "none":
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

        if args.visualizer == "meshcat":
            process_data_saver("meshcat")

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
            icp_viz_q[2] = 0.0

            icp_des_viz_q[0] = msg.des_icp[0]
            icp_des_viz_q[1] = msg.des_icp[1]
            icp_des_viz_q[2] = 0.0

            cmp_des_viz_q[0] = msg.des_cmp[0]
            cmp_des_viz_q[1] = msg.des_cmp[1]
            cmp_des_viz_q[2] = 0.0

            viz.display(vis_q)
            com_des_viz.display(com_des_viz_q)
            com_viz.display(com_viz_q)
            com_proj_viz.display(com_proj_viz_q)
            icp_viz.display(icp_viz_q)
            icp_des_viz.display(icp_des_viz_q)
            cmp_des_viz.display(cmp_des_viz_q)

            # plot GRFs
            if msg.phase != 1:
                vis_tools.grf_display(
                    arrow_viz["grf_lf"], msg.lfoot_pos, msg.lfoot_ori, msg.lfoot_rf_cmd
                )
                vis_tools.grf_display(
                    arrow_viz["grf_rf"], msg.rfoot_pos, msg.rfoot_ori, msg.rfoot_rf_cmd
                )

                # add sensed normal force
                l_force_local = quat_to_rot(msg.lfoot_ori) @ np.array(
                    [0.0, 0.0, msg.lfoot_rf_normal_filt]
                )
                r_force_local = quat_to_rot(msg.rfoot_ori) @ np.array(
                    [0.0, 0.0, msg.rfoot_rf_normal_filt]
                )
                lfoot_rf_normal = np.array(
                    [
                        0.0,
                        0.0,
                        0.0,
                        l_force_local[0],
                        l_force_local[1],
                        l_force_local[2],
                    ]
                )
                rfoot_rf_normal = np.array(
                    [
                        0.0,
                        0.0,
                        0.0,
                        r_force_local[0],
                        r_force_local[1],
                        r_force_local[2],
                    ]
                )
                vis_tools.grf_display(
                    arrow_viz["grf_lf_normal"],
                    msg.lfoot_pos,
                    msg.lfoot_ori,
                    lfoot_rf_normal,
                )
                vis_tools.grf_display(
                    arrow_viz["grf_rf_normal"],
                    msg.rfoot_pos,
                    msg.rfoot_ori,
                    rfoot_rf_normal,
                )
        elif args.visualizer == "foxglove":
            print("FLAG_FOXTRIG")
            # webbrowser.open('https://app.foxglove.dev/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765')
            th_fast = threading.Thread(target=asyncio.run(main()), args=())
            th_fast.start()
            # asyncio.run(main())

    else:  # if 'none' specified
        process_data_saver("none")

    # publish back to plot juggler
    # note: currently, this is not reached when using foxglove but the corresponding
    # ROS messages can be visualized within foxglove
    if args.b_use_plotjuggler:
        process_data_saver("none")
        pj_socket.send_string(json.dumps(data_saver.history))
