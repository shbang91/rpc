import os
import numpy as np
import pinocchio as pin
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform

from util.python_utils.util import rot_to_quat


def isMesh(geometry_object):
    """Check whether the geometry object contains a Mesh supported by MeshCat"""
    if geometry_object.meshPath == "":
        return False

    _, file_extension = os.path.splitext(geometry_object.meshPath)
    if file_extension.lower() in [".dae", ".obj", ".stl"]:
        return True

    return False


def update_robot_transform(
    visual: pin.GeometryModel,
    visual_data: pin.GeometryData,
    visual_model: pin.GeometryModel,
    transform: FrameTransform,
):
    parent_name = "world"
    # Get mesh pose.
    M = visual_data.oMg[visual_model.getGeometryId(visual.name)]
    # Manage scaling
    if isMesh(visual):
        scale = np.asarray(visual.meshScale).flatten()
        S = np.diag(np.concatenate((scale, [1.0])))
        T = np.array(M.homogeneous).dot(S)
    else:
        T = M.homogeneous
    visual_name = visual.name[:-2]
    transform.parent_frame_id = parent_name
    transform.child_frame_id = visual_name
    transform.translation.x = T[0][3]
    transform.translation.y = T[1][3]
    transform.translation.z = T[2][3]
    rot = T[:3, :3]
    q = rot_to_quat(rot)
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]


def update_2d_transform(obj_name: str, pos_2d: np.ndarray, transform: FrameTransform):
    transform.parent_frame_id = "world"
    transform.child_frame_id = obj_name
    transform.translation.x = pos_2d[0]
    transform.translation.y = pos_2d[1]
