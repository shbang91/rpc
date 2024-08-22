"""
Converts a pkl file to mcap file for visualization in Foxglove.
This assumes a specific set of parameters available in the pkl file,
such as time, base_pos, base_ori, joint_positions, icp_est, icp_des, etc.
"""
import os
import sys
import argparse
import pickle
import pinocchio as pin
import numpy as np

# foxglove + mcap tools for visualization
from mcap_protobuf.writer import Writer
from UI.visualization_toolbox import update_robot_transform, update_2d_transform
from foxglove_schemas_protobuf.Point2_pb2 import Point2
from foxglove_schemas_protobuf.Point3_pb2 import Point3
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate

cwd = os.getcwd()
sys.path.append(cwd)


def create_sphere_scene(scene_frame_id, rgba):
    icp_est_scene = SceneUpdate()
    icp_est_entity = icp_est_scene.entities.add()
    icp_est_entity.frame_id = scene_frame_id
    icp_est_model = icp_est_entity.spheres.add()
    icp_est_model.color.r = rgba[0]
    icp_est_model.color.g = rgba[1]
    icp_est_model.color.b = rgba[2]
    icp_est_model.color.a = rgba[3]
    icp_est_model.size.x = 0.02
    icp_est_model.size.y = 0.02
    icp_est_model.size.z = 0.02
    return icp_est_scene


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--b_using_kf_estimator", type=bool, default=True)
    args = parser.parse_args()
    b_using_kf_estimator = args.b_using_kf_estimator

    # variables in pkl file
    time =[]
    base_pos, base_ori, joint_positions = [], [], []
    vis_2d_object_names = ["icp_est", "icp_des"]
    vis_3d_object_names = ["lf_pos", "rf_pos"]

    # create dictionary for 2D plots
    vis_2d_dict = {}
    for oname in vis_2d_object_names:
        vis_2d_dict[oname] = []

    # create dictionary for 3D plots
    vis_3d_dict = {}
    for oname in vis_3d_object_names:
        vis_3d_dict[oname] = []

    # Read and collect all data from pkl file
    with open(cwd + "/experiment_data/pnc.pkl", "rb") as f:
        while True:
            try:
                d = pickle.load(f)
                time.append(d["time"])
                # Get mesh pose.
                if b_using_kf_estimator:
                    base_pos.append(d["kf_base_joint_pos"])
                    base_ori.append(d["kf_base_joint_ori"])
                else:
                    base_pos.append(d["est_base_joint_pos"])
                    base_ori.append(d["est_base_joint_ori"])
                joint_positions.append(d["joint_positions"])
                vis_2d_dict["icp_est"].append(d["est_icp"])
                vis_2d_dict["icp_des"].append(d["des_icp"])
                vis_3d_dict["lf_pos"].append(d["lfoot_pos"])
                vis_3d_dict["rf_pos"].append(d["rfoot_pos"])

            except EOFError:
                break

    # Load pinocchio model and data of Draco3
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        "robot_model/draco/draco_modified.urdf",
        "robot_model/draco",
        pin.JointModelFreeFlyer(),
    )
    data, collision_data, visual_data = pin.createDatas(
        model, collision_model, visual_model
    )
    vis_q = pin.neutral(model)
    transform = FrameTransform()

    icp_est_scene = create_sphere_scene("icp_est", [1.0, 0.0, 0.0, 0.5])
    icp_des_scene = create_sphere_scene("icp_des", [0.0, 1.0, 0.0, 0.5])

    # send data to mcap file
    with open(cwd + "/experiment_data/draco3_foxglove.mcap", "wb") as f, Writer(f) as mcap_writer:
        for i in range(len(time)):
            # Update all transforms (to visualize URDF)
            vis_q[0:3] = np.array(base_pos[i])
            vis_q[3:7] = np.array(base_ori[i])  # quaternion [x,y,z,w]
            vis_q[7:] = np.array(joint_positions[i])
            # ===========================================
            # update transformations of all visual model objects
            pin.forwardKinematics(model, data, vis_q)
            pin.updateGeometryPlacements(model, data, visual_model, visual_data)
            for visual in visual_model.geometryObjects:
                update_robot_transform(visual, visual_data, visual_model, transform)
                mcap_writer.write_message("transforms", transform, int(time[i] * 1e9), int(time[i] * 1e9))
                transform.rotation.Clear()
                transform.translation.Clear()
            # -------------------------------------------
            # update transform of ADDITIONAL visual elements
            for vname, vval in vis_2d_dict.items():
                update_2d_transform(vname, vval[i], transform)
                mcap_writer.write_message( "transforms", transform,
                                           int(time[i] * 1e9), int(time[i] * 1e9))
                transform.rotation.Clear()
                transform.translation.Clear()
            # ===========================================
            # ICP visuals (est / des)
            icp_est_scene.entities[0].timestamp.FromNanoseconds(int(time[i] * 1e9))
            mcap_writer.write_message( "icp_est_viz", icp_est_scene,
                                       int(time[i] * 1e9), int(time[i] * 1e9))
            # -------------------------------------------
            icp_des_scene.entities[0].timestamp.FromNanoseconds(int(time[i] * 1e9))
            mcap_writer.write_message( "icp_des_viz", icp_des_scene,
                                       int(time[i] * 1e9), int(time[i] * 1e9))
            # ===========================================
            # ICP plots (est / des)
            for vname, vval in vis_2d_dict.items():
                mcap_writer.write_message(vname,
                                          Point2(x=vval[i][0], y=vval[i][1]),
                                          int(time[i] * 1e9),
                                          int(time[i] * 1e9))
            # Feet position plots (left / right)
            for vname, vval in vis_3d_dict.items():
                mcap_writer.write_message(vname,
                                          Point3(x=vval[i][0], y=vval[i][1], z=vval[i][2]),
                                          int(time[i] * 1e9),
                                          int(time[i] * 1e9))
        mcap_writer.finish()


if __name__ == "__main__":
    main()