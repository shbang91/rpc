import os
import sys
import zmq
import time
import ruamel.yaml as yaml
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build")

# NEED TO GET THIS BACK --- FIX LATER
from util.python_utils.util import rot_to_quat, quat_to_rot
from messages.draco_pb2 import *

from plot.data_saver import *
import pinocchio as pin
import json
import argparse
from scipy.spatial.transform import Rotation as R


parser = argparse.ArgumentParser()
parser.add_argument("--b_use_plotjuggler", type=bool, default=False)
parser.add_argument(
    "--visualizer", choices=["none", "meshcat", "foxglove"], default="none"
)
parser.add_argument(
    "--robot", choices=["draco", "fixe_draco", "manipulator"], default="draco"
)
parser.add_argument("--hw_or_sim", choices=["hw", "sim"], default="sim")
args = parser.parse_args()

# Set max number of planned footstep visuals
STEP_MAX = 20

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
    import footstep_planner as fp
    from watchdog.observers import Observer

    # local tools to manage Foxglove scenes
    from plot.foxglove_utils import SceneChannel, ShapeScene
    from UI.visualization_toolbox import update_robot_transform

    # load parameters that can be controlled / changed and start Control Parameters server
    param_store = foxglove_ctrl.load_params_store()
    step_listener = foxglove_ctrl.Listener(param_store)
    th_slow = threading.Thread(
        target=asyncio.run, args=([foxglove_ctrl.run(step_listener)])
    )
    th_slow.start()
    time.sleep(20.0)  # give some time for state estimator to publish data

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

##==========================================================================
## Load config file
##==========================================================================
with open("config/" + args.robot + "/INTERFACE.yaml", "r") as iface_yaml:
    try:
        config = yaml.safe_load(iface_yaml)
        env = config["test_env_name"]
        wbc = config["whole_body_controller"]
    except yaml.YAMLError as exc:
        print(exc)

pnc_path = (
    "config/" + args.robot + "/" + args.hw_or_sim + "/" + env + "/" + wbc + "/pnc.yaml"
)

grf_names = ["lfoot_rf_cmd", "rfoot_rf_cmd"]
# "lfoot_rf_normal_filt", "rfoot_rf_normal_filt"]

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


def foot_dimensions():
    with open(pnc_path, "r") as pnc_stream:
        try:
            pnc_cfg = yaml.safe_load(pnc_stream)
            foot_half_length = pnc_cfg["wbc"]["contact"]["foot_half_length"]
            foot_half_width = pnc_cfg["wbc"]["contact"]["foot_half_width"]
        except yaml.YAMLError as exc:
            print(exc)
    return foot_half_length, foot_half_width


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
        icp_est_chan_id = await SceneChannel(
            True, "icp_est", "json", "icp_est", ["x", "y"]
        ).add_chan(server)
        icp_des_chan_id = await SceneChannel(
            True, "icp_des", "json", "icp_des", ["x", "y"]
        ).add_chan(server)
        # proj_footstepS_chan_id = await SceneChannel(
        #     False,
        #     "proj_footstep_viz",
        #     "protobuf",
        #     SceneUpdate.DESCRIPTOR.full_name,
        #     scene_schema,
        # ).add_chan(server)
        b_ft_contact_chan_id = await SceneChannel(
            True, "b_ft_contact", "json", "b_ft_contact", ["b_lfoot", "b_rfoot"]
        ).add_chan(server)
        lf_pos_chan_id = await SceneChannel(
            True, "lf_pos", "json", "lf_pos", ["x", "y", "z"]
        ).add_chan(server)
        rf_pos_chan_id = await SceneChannel(
            True, "rf_pos", "json", "rf_pos", ["x", "y", "z"]
        ).add_chan(server)

        for scn in range(len(xyz_scene_names)):
            await sceneinitman(xyz_scene_names[scn], server)

        # STEP_MAX available footstep plans
        hfoot_length, hfoot_width = foot_dimensions()
        x = 2 * hfoot_length
        y = 2 * hfoot_width
        msgs, size, color = [], {}, {}
        proj_footstep_chan_ids = []
        proj_footstep_viz_chan_ids = []
        proj_foot_pos, proj_foot_ori = {}, {}  # Stores step position
        proj_feet = []
        for i in range(STEP_MAX):
            rf = "proj_rf" + str(i)
            lf = "proj_lf" + str(i)
            msgs.append(rf)
            msgs.append(lf)
            size[rf], size[lf] = [x, y, 0.001], [x, y, 0.001]
            color[rf] = [1, 0, 0, 1]
            color[lf] = [0.1, 0.5, 1, 1]
            proj_foot_pos[rf], proj_foot_pos[lf] = [0, 0, 0], [0, 0.184, 0]
            proj_foot_ori[rf], proj_foot_ori[lf] = [0, 0, 0, 0], [0, 0, 0, 0]
            if i < 10:
                name = "projected_footsteps_0" + str(i)
            else:
                name = "projected_footsteps_" + str(i)
            # Add a channel for projected footstep data
            foot_scene = await SceneChannel(
                True,
                name,
                "json",
                name,
                [
                    "rf_pos_x",
                    "rf_pos_y",
                    "rf_pos_z",
                    "rf_ori_x",
                    "rf_ori_y",
                    "rf_ori_z",
                    "rf_ori_q",
                    "lf_pos_x",
                    "lf_pos_y",
                    "lf_pos_z",
                    "lf_ori_x",
                    "lf_ori_y",
                    "lf_ori_z",
                    "lf_ori_q",
                ],
            ).add_chan(server)
            proj_footstep_chan_ids.append(foot_scene)

            # Add a channel for projected footstep visual elements
            proj_footstep_viz_chan_id = await SceneChannel(
                False,
                name + "_viz",
                "protobuf",
                SceneUpdate.DESCRIPTOR.full_name,
                scene_schema,
            ).add_chan(server)
            proj_footstep_viz_chan_ids.append(proj_footstep_viz_chan_id)
            # add footstep visual elements
            proj_footsteps = ShapeScene()
            proj_footsteps.add_shape(rf, "cubes", [1, 0, 0, 1], [x, y, 0.001])
            proj_footsteps.add_shape(lf, "cubes", [0.1, 0.5, 1, 1], [x, y, 0.001])
            proj_feet.append(proj_footsteps)

        arrows_scene = ShapeScene()
        arrows_scene.add_shape(
            "lfoot_rf_cmd", "arrows", [0, 0, 1, 0.5], [0.03, 0.1, 0.08]
        )  # blue arrow
        arrows_scene.add_shape(
            "rfoot_rf_cmd", "arrows", [0, 0, 1, 0.5], [0.03, 0.1, 0.08]
        )  # blue arrow
        arrows_scene.add_shape(
            "lfoot_rf_normal_filt", "arrows", [0.2, 0.2, 0.2, 0.5], [0.03, 0.1, 0.08]
        )  # grey arrow
        arrows_scene.add_shape(
            "rfoot_rf_normal_filt", "arrows", [0.2, 0.2, 0.2, 0.5], [0.03, 0.1, 0.08]
        )  # grey arrow

        # add visual icp spheres
        icp_spheres = ShapeScene()
        icp_spheres.add_shape("est_icp", "spheres", [1, 0, 1, 1], [0.03, 0.03, 0.03])
        icp_spheres.add_shape("des_icp", "spheres", [0, 1, 0, 1], [0.03, 0.03, 0.03])

        # Send the FrameTransform every frame to update the model's position
        transform = FrameTransform()

        print("foxglove websocket initiated")

        # Clear experiment_data to start new footstep planning
        fp.remove_yaml_files(fp.WATCHED_DIR)
        # Initialization for footstep planning  -- interrupts upon yaml creation
        event_handler = fp.yamlHandler()
        observer = Observer()
        observer.schedule(event_handler, path=fp.WATCHED_DIR, recursive=False)
        observer.start()

        while True:
            tasks = []  # Scenes to synchronously update

            # receive msg trough socket
            encoded_msg = socket.recv()
            msg.ParseFromString(encoded_msg)
            check_if_kf_estimator(msg.kf_base_joint_pos, msg.est_base_joint_pos)

            await asyncio.sleep(0.01)
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
                icp_est_chan_id,
                now,
                json.dumps(
                    {
                        "x": list(msg.est_icp)[0],
                        "y": list(msg.est_icp)[1],
                    }
                ).encode("utf8"),
            )
            await server.send_message(
                icp_des_chan_id,
                now,
                json.dumps(
                    {
                        "x": list(msg.des_icp)[0],
                        "y": list(msg.des_icp)[1],
                    }
                ).encode("utf8"),
            )

            await server.send_message(
                b_ft_contact_chan_id,
                now,
                json.dumps(
                    {
                        "b_lfoot": msg.b_lfoot,
                        "b_rfoot": msg.b_rfoot,
                    }
                ).encode("utf8"),
            )

            await server.send_message(
                lf_pos_chan_id,
                now,
                json.dumps(
                    {
                        "x": list(msg.lfoot_pos)[0],
                        "y": list(msg.lfoot_pos)[1],
                        "z": list(msg.lfoot_pos)[2],
                    }
                ).encode("utf8"),
            )

            await server.send_message(
                rf_pos_chan_id,
                now,
                json.dumps(
                    {
                        "x": list(msg.rfoot_pos)[0],
                        "y": list(msg.rfoot_pos)[1],
                        "z": list(msg.rfoot_pos)[2],
                    }
                ).encode("utf8"),
            )

            for idx in range(STEP_MAX):
                rf = "proj_rf" + str(idx)
                lf = "proj_lf" + str(idx)
                r_pos = proj_foot_pos[rf]
                l_pos = proj_foot_pos[lf]
                r_ori = proj_foot_ori[rf]
                l_ori = proj_foot_ori[lf]
                await server.send_message(
                    proj_footstep_chan_ids[idx],
                    now,
                    json.dumps(
                        {
                            "rf_pos_x": r_pos[0],
                            "rf_pos_y": r_pos[1],
                            "rf_pos_z": r_pos[2],
                            "rf_ori_x": r_ori[1],
                            "rf_ori_y": r_ori[2],
                            "rf_ori_z": r_ori[3],
                            "rf_ori_q": r_ori[0],
                            "lf_pos_x": l_pos[0],
                            "lf_pos_y": l_pos[1],
                            "lf_pos_z": l_pos[2],
                            "lf_ori_x": l_ori[1],
                            "lf_ori_y": l_ori[2],
                            "lf_ori_z": l_ori[3],
                            "lf_ori_q": l_ori[0],
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
            pin.forwardKinematics(model, data, vis_q)
            pin.updateGeometryPlacements(model, data, visual_model, visual_data)
            for visual in visual_model.geometryObjects:
                update_robot_transform(visual, visual_data, visual_model, transform)
                await server.send_message(
                    tf_chan_id, now, transform.SerializeToString()
                )
                transform.rotation.Clear()
                transform.translation.Clear()

            Ry = R.from_euler("y", -np.pi / 2).as_matrix()
            # update GRF arrows
            for obj in grf_names:
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
                arrows_scene.scale(obj, quat_force, force_magnitude, now)
                tasks.append(
                    server.send_message(tf_chan_id, now, transform.SerializeToString())
                )
                await server.send_message(
                    normS_chan_id, now, arrows_scene.serialized_msg(obj)
                )

            # update icp values on the grid
            for obj in ["est_icp", "des_icp"]:
                transform.parent_frame_id = "world"
                transform.child_frame_id = obj
                transform.timestamp.FromNanoseconds(now)
                transform.translation.x = list(getattr(msg, obj))[0]
                transform.translation.y = list(getattr(msg, obj))[1]
                icp_spheres.update(obj, now)
                tasks.append(
                    server.send_message(tf_chan_id, now, transform.SerializeToString())
                )
                tasks.append(
                    server.send_message(
                        icpS_chan_id, now, icp_spheres.serialized_msg(obj)
                    )
                )

            # Update projected footsteps
            update = fp.sd.steps_to_update()
            for obj in msgs:
                if obj in update:
                    stepnum = int("".join(filter(lambda i: i.isdigit(), obj)))
                    yaml = fp.sd.yaml_num
                    setattr(
                        fp.sd,
                        obj[5] + "f_steps_taken",
                        getattr(fp.sd, obj[5] + "f_steps_taken") + 1,
                    )
                    proj_foot_pos[obj] = getattr(fp, obj[5] + "foot_contact_pos")[yaml][
                        update[obj]
                    ]
                    proj_foot_ori[obj] = getattr(fp, obj[5] + "foot_contact_ori")[yaml][
                        update[obj]
                    ]
                    transform.parent_frame_id = "world"
                    transform.child_frame_id = obj
                    transform.timestamp.FromNanoseconds(now)
                    transform.translation.x = proj_foot_pos[obj][0]
                    transform.translation.y = proj_foot_pos[obj][1]
                    transform.translation.z = proj_foot_pos[obj][2]
                    transform.rotation.x = proj_foot_ori[obj][1]
                    transform.rotation.y = proj_foot_ori[obj][2]
                    transform.rotation.z = proj_foot_ori[obj][3]
                    transform.rotation.w = proj_foot_ori[obj][0]
                    proj_feet[stepnum].update(obj, now)
                    tasks.append(
                        server.send_message(
                            tf_chan_id, now, transform.SerializeToString()
                        )
                    )
                    tasks.append(
                        server.send_message(
                            proj_footstep_viz_chan_ids[stepnum],
                            now,
                            proj_feet[stepnum].serialized_msg(obj),
                        )
                    )

            await asyncio.gather(*tasks)


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
with open(pnc_path) as yaml_file:
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
