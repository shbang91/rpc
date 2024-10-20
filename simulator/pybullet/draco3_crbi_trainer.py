import os
import sys
import meshcat
import copy
import shutil
import matplotlib.pyplot as plt
import numpy as np

from plot.data_saver import DataSaver
from tqdm import tqdm
from ruamel.yaml import YAML
from casadi import *
from util.python_utils import util
from util.python_utils import interpolation
from util.python_utils import liegroup
from enum import Enum

# kinematics tools
import qpsolvers
import pinocchio as pin
import meshcat_shapes
import pink
from pink import solve_ik
from pink.tasks import FrameTask, JointCouplingTask, PostureTask
from loop_rate_limiters import RateLimiter

# training libraries / tools
import torch
import torch.utils.data as torch_utils
from torch.utils.tensorboard import SummaryWriter

cwd = os.getcwd()
sys.path.append(cwd)
np.set_printoptions(precision=3)

## Configs
VISUALIZE = False
VIDEO_RECORD = False
SAVE_DATA = False
PRINT_FREQ = 10
DT = 0.01
PRINT_ROBOT_INFO = False
INITIAL_POS_WORLD_TO_BASEJOINT = [0, 0, 1.5 - 0.761]  # 0.761, 0.681
INITIAL_QUAT_WORLD_TO_BASEJOINT = [0.0, 0.0, 0.0, 1.0]

## Motion Boundaries
RFOOT_POS_LB = np.array([-0.15, -0.1, -0.05])
RFOOT_POS_UB = np.array([0.15, 0.05, 0.05])
LFOOT_POS_LB = np.array([-0.15, -0.05, -0.05])
LFOOT_POS_UB = np.array([0.15, 0.1, 0.05])

FOOT_EA_LB = np.array([np.deg2rad(-5.0), np.deg2rad(-15.0), -np.pi / 4.0])
FOOT_EA_UB = np.array([np.deg2rad(5.0), np.deg2rad(15.0), np.pi / 4.0])

SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.05, 0.30

SWING_TIME_LB, SWING_TIME_UB = 0.35, 0.75

BASE_HEIGHT_LB, BASE_HEIGHT_UB = 0.7, 0.8

# Thresholds
MAX_HOL_ERROR = 0.2  # [rad] discrepancy between proximal and distal joints

## Dataset Generation
N_CPU_DATA_GEN = 4
N_DATA_PER_MOTION = 30
N_MOTION_PER_LEG = 2100 / N_DATA_PER_MOTION  # number of (forward) steps per leg
N_TURN_STEPS = 4
N_TURN_ITERS = 1e2  # number of iterations of each turning sequence

N_LAYER_OUTPUT = [64, 64]
LR = 0.01
MOMENTUM = 0.0
N_EPOCH = 20
BATCH_SIZE = 32
NN_INPUT_DIM = 12  # end-effector linear (xyz) and angular (rpy) position
NN_OUTPUT_DIM = 6  # Ixx, Iyy, Izz, Ixy, Ixz, Iyz

if SAVE_DATA:
    data_saver = DataSaver("draco3_crbi_fwd_step_joints.pkl")


class MotionType(Enum):
    STEP = 1
    TURN = 2


## 2-hidden layer Neural Network
class NetWork(torch.nn.Module):
    def __init__(self, n_input, n_hidden_1, n_hidden_2, n_output):
        super(NetWork, self).__init__()
        self.layers = torch.nn.Sequential(
            torch.nn.Linear(n_input, n_hidden_1),
            torch.nn.Tanh(),
            torch.nn.Linear(n_hidden_1, n_hidden_2),
            torch.nn.Tanh(),
            torch.nn.Linear(n_hidden_2, n_output),
        )

    def forward(self, x):
        return self.layers(x)


def display_visualizer_frames(meshcat_visualizer, frame, pin_geom_type=None):
    # assume visual mode, if unspecified
    if pin_geom_type is None:
        pin_geom_type = pin.GeometryType.VISUAL
        meshcat_model = meshcat_visualizer.visual_model
    elif pin_geom_type == pin.GeometryType.VISUAL:
        meshcat_model = meshcat_visualizer.visual_model
    elif pin_geom_type == pin.GeometryType.COLLISION:
        meshcat_model = meshcat_visualizer.collision_model
    else:
        raise ValueError("Invalid pinocchio GeometryType")

    for visual in meshcat_model.geometryObjects:
        # Get mesh pose.
        if pin_geom_type == pin.GeometryType.VISUAL:
            M = meshcat_visualizer.visual_data.oMg[
                meshcat_model.getGeometryId(visual.name)
            ]
        else:
            M = meshcat_visualizer.collision_data.oMg[
                meshcat_model.getGeometryId(visual.name)
            ]
        # Manage scaling
        scale = np.asarray(visual.meshScale).flatten()
        S = np.diag(np.concatenate((scale, [1.0])))
        T = np.array(M.homogeneous).dot(S)
        # Update viewer configuration.
        frame[
            meshcat_visualizer.getViewerNodeName(visual, pin_geom_type)
        ].set_transform(T)


def sample_swing_config(nominal_lf_iso, nominal_rf_iso, side, min_step_length=0.1):
    swing_time = np.random.uniform(SWING_TIME_LB, SWING_TIME_UB)
    min_y_distance_btw_foot = abs(
        nominal_lf_iso.translation[1] - nominal_rf_iso.translation[1]
    )
    # TODO remove samples with self-collisions?
    if side == "left":
        # Sample rfoot config
        rfoot_ini_iso = np.copy(nominal_rf_iso)
        rfoot_mid_iso = np.copy(nominal_rf_iso)
        rfoot_fin_iso = np.copy(nominal_rf_iso)

        rfoot_mid_vel = (rfoot_ini_iso[0:3, 3] - rfoot_ini_iso[0:3, 3]) / swing_time
        rfoot_mid_vel[2] = 0.0

        # Sample lfoot config
        lfoot_ini_pos = np.copy(nominal_lf_iso)[0:3, 3] + np.random.uniform(
            np.array([0.0, LFOOT_POS_LB[1], LFOOT_POS_LB[2]]),
            np.array([0.0, LFOOT_POS_UB[1], LFOOT_POS_UB[2]]),
        )
        lfoot_ini_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        lfoot_ini_rot = util.euler_to_rot(lfoot_ini_ea)
        lfoot_ini_iso = liegroup.RpToTrans(lfoot_ini_rot, lfoot_ini_pos)
        lfoot_fin_pos = np.copy(nominal_lf_iso)[0:3, 3] + np.random.uniform(
            LFOOT_POS_LB, LFOOT_POS_UB
        )

        # resample until min distance in y-direction is satisfied
        while (
            abs(lfoot_ini_pos[1] - nominal_rf_iso.translation[1])
            < min_y_distance_btw_foot
        ):
            lfoot_ini_pos = nominal_lf_iso.translation + np.random.uniform(
                LFOOT_POS_LB, LFOOT_POS_UB
            )
        while (
            abs(lfoot_fin_pos[1] - nominal_rf_iso.translation[1])
            < min_y_distance_btw_foot
        ):
            lfoot_fin_pos = nominal_lf_iso.translation + np.random.uniform(
                LFOOT_POS_LB, LFOOT_POS_UB
            )

        # modify x-direction sample until min distance in x-direction is satisfied
        while abs(lfoot_fin_pos[0] - nominal_rf_iso.translation[0]) < min_step_length:
            lfoot_fin_pos[0] = lfoot_fin_pos[0] + np.random.uniform(
                0.0, LFOOT_POS_UB[0]
            )

        lfoot_fin_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        lfoot_fin_rot = util.euler_to_rot(lfoot_fin_ea)
        lfoot_fin_iso = liegroup.RpToTrans(lfoot_fin_rot, lfoot_fin_pos)
        lfoot_mid_iso = interpolation.iso_interpolate(lfoot_ini_iso, lfoot_fin_iso, 0.5)
        lfoot_mid_iso[2, 3] = (
            lfoot_ini_pos[2] + lfoot_fin_pos[2]
        ) / 2.0 + np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

        lfoot_mid_vel = (lfoot_fin_pos - lfoot_ini_pos) / swing_time
        lfoot_mid_vel[2] = 0.0

        # Sample base config
        base_ini_pos = (rfoot_ini_iso[0:3, 3] + lfoot_ini_iso[0:3, 3]) / 2.0
        base_ini_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_ini_rot = util.euler_to_rot(np.array([0.0, 0.0, lfoot_ini_ea[2] / 2.0]))
        base_ini_iso = liegroup.RpToTrans(base_ini_rot, base_ini_pos)
        base_fin_pos = (rfoot_fin_iso[0:3, 3] + lfoot_fin_iso[0:3, 3]) / 2.0
        base_fin_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_fin_rot = util.euler_to_rot(np.array([0.0, 0.0, lfoot_fin_ea[2] / 2.0]))
        base_fin_iso = liegroup.RpToTrans(base_fin_rot, base_fin_pos)

    elif side == "right":
        # Sample lfoot config
        lfoot_ini_iso = np.copy(nominal_lf_iso)
        lfoot_mid_iso = np.copy(nominal_lf_iso)
        lfoot_fin_iso = np.copy(nominal_lf_iso)

        lfoot_mid_vel = (lfoot_ini_iso[0:3, 3] - lfoot_ini_iso[0:3, 3]) / swing_time
        lfoot_mid_vel[2] = 0.0

        # Sample rfoot config
        rfoot_ini_pos = np.copy(nominal_rf_iso)[0:3, 3] + np.random.uniform(
            RFOOT_POS_LB, RFOOT_POS_UB
        )
        rfoot_ini_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        rfoot_ini_rot = util.euler_to_rot(rfoot_ini_ea)
        rfoot_ini_iso = liegroup.RpToTrans(rfoot_ini_rot, rfoot_ini_pos)
        rfoot_fin_pos = np.copy(nominal_rf_iso)[0:3, 3] + np.random.uniform(
            RFOOT_POS_LB, RFOOT_POS_UB
        )

        # resample until min distance in y-direction is satisfied
        while (
            abs(rfoot_ini_pos[1] - nominal_lf_iso.translation[1])
            < min_y_distance_btw_foot
        ):
            rfoot_ini_pos = nominal_rf_iso.translation + np.random.uniform(
                RFOOT_POS_LB, RFOOT_POS_UB
            )
        while (
            abs(rfoot_fin_pos[1] - nominal_lf_iso.translation[1])
            < min_y_distance_btw_foot
        ):
            rfoot_fin_pos = nominal_rf_iso.translation + np.random.uniform(
                RFOOT_POS_LB, RFOOT_POS_UB
            )

        # modify x-direction sample until min distance in x-direction is satisfied
        while abs(rfoot_fin_pos[0] - nominal_lf_iso.translation[0]) < min_step_length:
            rfoot_fin_pos[0] = rfoot_fin_pos[0] + np.random.uniform(
                0.0, RFOOT_POS_UB[0]
            )

        rfoot_fin_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        rfoot_fin_rot = util.euler_to_rot(rfoot_fin_ea)
        rfoot_fin_iso = liegroup.RpToTrans(rfoot_fin_rot, rfoot_fin_pos)
        rfoot_mid_iso = interpolation.iso_interpolate(rfoot_ini_iso, rfoot_fin_iso, 0.5)
        rfoot_mid_iso[2, 3] = (
            rfoot_ini_pos[2] + rfoot_fin_pos[2]
        ) / 2.0 + np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

        rfoot_mid_vel = (rfoot_fin_pos - rfoot_ini_pos) / swing_time
        rfoot_mid_vel[2] = 0.0

        # Sample base config
        base_ini_pos = (rfoot_ini_iso[0:3, 3] + lfoot_ini_iso[0:3, 3]) / 2.0
        base_ini_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_ini_rot = util.euler_to_rot(np.array([0.0, 0.0, rfoot_ini_ea[2] / 2.0]))
        base_ini_iso = liegroup.RpToTrans(base_ini_rot, base_ini_pos)
        base_fin_pos = (rfoot_fin_iso[0:3, 3] + lfoot_fin_iso[0:3, 3]) / 2.0
        base_fin_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_fin_rot = util.euler_to_rot(np.array([0.0, 0.0, rfoot_fin_ea[2] / 2.0]))
        base_fin_iso = liegroup.RpToTrans(base_fin_rot, base_fin_pos)

    else:
        raise ValueError

    return (
        swing_time,
        lfoot_ini_iso,
        lfoot_mid_iso,
        lfoot_fin_iso,
        lfoot_mid_vel,
        rfoot_ini_iso,
        rfoot_mid_iso,
        rfoot_fin_iso,
        rfoot_mid_vel,
        base_ini_iso,
        base_fin_iso,
    )


def sample_turn_config(
    prev_base_iso, prev_lf_iso, prev_rf_iso, cw_or_ccw, swing_leg="right_leg"
):
    swing_time = np.random.uniform(SWING_TIME_LB, SWING_TIME_UB)

    if swing_leg == "right_leg":
        # Keep old (left) stance leg config
        lfoot_ini_iso = np.copy(prev_lf_iso)
        lfoot_mid_iso = np.copy(prev_lf_iso)
        lfoot_fin_iso = np.copy(prev_lf_iso)

        lfoot_mid_vel = (lfoot_ini_iso[0:3, 3] - lfoot_ini_iso[0:3, 3]) / swing_time
        lfoot_mid_vel[2] = 0.0

        if cw_or_ccw == "cw":
            # our ref. frame is stance foot, so local y-axis bounds change sign
            T_SGN = np.array([1.0, -1.0, 1.0])

            # Keep initial (rfoot) swing (x,y)-pos config same as nominal
            rfoot_ini_iso = np.copy(prev_rf_iso)
            rfoot_ini_pos = rfoot_ini_iso[-1, :3]

            # Sample (lfoot) swing final config (currently not avoiding self-collisions)
            w_R_lf = lfoot_ini_iso[0:3, 0:3]
            rfoot_fin_pos = np.copy(lfoot_ini_iso[:3, 3]) + w_R_lf @ np.random.uniform(
                T_SGN * TURN_STANCE_POS_LB, T_SGN * TURN_STANCE_POS_UB
            )

            stance_foot_rpy = util.rot_to_rpy(lfoot_ini_iso[0:3, 0:3])
            rfoot_fin_ea = np.random.uniform(
                np.array([-FOOT_EA_UB[0], -FOOT_EA_UB[1], -FOOT_EA_UB[2]]),
                np.array([-FOOT_EA_LB[0], -FOOT_EA_LB[0], 0.0]),
            )
            rfoot_fin_rot = util.euler_to_rot(
                [rfoot_fin_ea[0], rfoot_fin_ea[1], stance_foot_rpy[2] + rfoot_fin_ea[2]]
            )
            rfoot_fin_iso = liegroup.RpToTrans(rfoot_fin_rot, rfoot_fin_pos)
            rfoot_mid_iso = interpolation.iso_interpolate(
                rfoot_ini_iso, rfoot_fin_iso, 0.5
            )
            rfoot_mid_iso[2, 3] = (
                rfoot_ini_pos[2] + rfoot_fin_pos[2]
            ) / 2.0 + np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

            rfoot_mid_vel = (rfoot_fin_pos - rfoot_ini_pos) / swing_time
            rfoot_mid_vel[2] = 0.0

        else:
            # Keep initial (rfoot) swing (x,y)-pos config same as nominal
            rfoot_ini_iso = np.copy(prev_rf_iso)
            rfoot_ini_pos = rfoot_ini_iso[-1, :3]

            # Sample (rfoot) swing final config (currently not avoiding self-collisions)
            w_R_lf = lfoot_ini_iso[0:3, 0:3]
            rfoot_fin_pos = np.copy(lfoot_ini_iso[:3, 3]) + w_R_lf @ np.random.uniform(
                TURN_SWING_POS_LB, TURN_SWING_POS_UB
            )

            stance_foot_rpy = util.rot_to_rpy(lfoot_ini_iso[0:3, 0:3])
            rfoot_fin_ea = np.random.uniform(
                np.array([FOOT_EA_LB[0], FOOT_EA_LB[1], 0.5]), FOOT_EA_UB
            )
            rfoot_fin_rot = util.euler_to_rot(
                [rfoot_fin_ea[0], rfoot_fin_ea[1], stance_foot_rpy[2] + rfoot_fin_ea[2]]
            )
            rfoot_fin_iso = liegroup.RpToTrans(rfoot_fin_rot, rfoot_fin_pos)
            rfoot_mid_iso = interpolation.iso_interpolate(
                rfoot_ini_iso, rfoot_fin_iso, 0.5
            )
            rfoot_mid_iso[2, 3] = (
                rfoot_ini_pos[2] + rfoot_fin_pos[2]
            ) / 2.0 + np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

            rfoot_mid_vel = (rfoot_fin_pos - rfoot_ini_pos) / swing_time
            rfoot_mid_vel[2] = 0.0

    elif swing_leg == "left_leg":
        # Keep old (right) stance leg config
        rfoot_ini_iso = np.copy(prev_rf_iso)
        rfoot_mid_iso = np.copy(prev_rf_iso)
        rfoot_fin_iso = np.copy(prev_rf_iso)

        rfoot_mid_vel = (rfoot_ini_iso[0:3, 3] - rfoot_ini_iso[0:3, 3]) / swing_time
        rfoot_mid_vel[2] = 0.0

        if cw_or_ccw == "cw":
            # our ref. frame is stance foot, so local y-axis bounds change sign
            T_SGN = np.array([1.0, -1.0, 1.0])

            # Sample rfoot config
            # Keep initial (rfoot) swing (x,y)-pos config same as nominal
            lfoot_ini_iso = np.copy(prev_lf_iso)
            lfoot_ini_pos = lfoot_ini_iso[-1, :3]

            # Sample (rfoot) swing final config (currently not avoiding self-collisions)
            w_R_rf = rfoot_ini_iso[0:3, 0:3]
            lfoot_fin_pos = np.copy(rfoot_ini_iso[:3, 3]) + w_R_rf @ np.random.uniform(
                T_SGN * TURN_SWING_POS_LB, T_SGN * TURN_SWING_POS_UB
            )  # TODO check this

            stance_foot_rpy = util.rot_to_rpy(rfoot_ini_iso[0:3, 0:3])
            lfoot_fin_ea = np.random.uniform(
                np.array([-FOOT_EA_UB[0], -FOOT_EA_UB[1], -FOOT_EA_UB[2]]),
                np.array([-FOOT_EA_LB[0], -FOOT_EA_LB[1], -0.5]),
            )
            lfoot_fin_rot = util.euler_to_rot(
                [lfoot_fin_ea[0], lfoot_fin_ea[1], stance_foot_rpy[2] + lfoot_fin_ea[2]]
            )
            lfoot_fin_iso = liegroup.RpToTrans(lfoot_fin_rot, lfoot_fin_pos)
            lfoot_mid_iso = interpolation.iso_interpolate(
                lfoot_ini_iso, lfoot_fin_iso, 0.5
            )
            lfoot_mid_iso[2, 3] = (
                lfoot_ini_pos[2] + lfoot_fin_pos[2]
            ) / 2.0 + np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

            lfoot_mid_vel = (lfoot_fin_pos - lfoot_ini_pos) / swing_time
            lfoot_mid_vel[2] = 0.0

        else:  # left leg when turning counter-clockwise
            # Keep initial (lfoot) swing (x,y)-pos config same as nominal
            lfoot_ini_iso = np.copy(prev_lf_iso)
            lfoot_ini_pos = lfoot_ini_iso[-1, :3]

            # Sample (lfoot) swing final config (currently not avoiding self-collisions)
            w_R_rf = rfoot_ini_iso[0:3, 0:3]
            lfoot_fin_pos = np.copy(rfoot_ini_iso[:3, 3]) + w_R_rf @ np.random.uniform(
                TURN_STANCE_POS_LB, TURN_STANCE_POS_UB
            )

            stance_foot_rpy = util.rot_to_rpy(rfoot_ini_iso[0:3, 0:3])
            lfoot_fin_ea = np.random.uniform(
                np.array([FOOT_EA_LB[0], FOOT_EA_LB[1], 0.0]), FOOT_EA_UB
            )
            lfoot_fin_rot = util.euler_to_rot(
                [lfoot_fin_ea[0], lfoot_fin_ea[1], stance_foot_rpy[2] + lfoot_fin_ea[2]]
            )
            lfoot_fin_iso = liegroup.RpToTrans(lfoot_fin_rot, lfoot_fin_pos)
            lfoot_mid_iso = interpolation.iso_interpolate(
                lfoot_ini_iso, lfoot_fin_iso, 0.5
            )
            lfoot_mid_iso[2, 3] = (
                lfoot_ini_pos[2] + lfoot_fin_pos[2]
            ) / 2.0 + np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

            lfoot_mid_vel = (lfoot_fin_pos - lfoot_ini_pos) / swing_time
            lfoot_mid_vel[2] = 0.0
    else:
        raise ValueError(f"Invalid swing_leg: {swing_leg}")

    # Sample base config
    base_ini_iso = np.copy(prev_base_iso)
    base_fin_pos = (rfoot_fin_iso[:3, 3] + lfoot_fin_iso[:3, 3]) / 2.0
    base_fin_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)

    # wrap rotations in positive direction
    l_foot_yaw = util.rot_to_rpy(lfoot_fin_iso[:3, :3])[2]
    if l_foot_yaw < 0:
        l_foot_yaw += 2 * np.pi
    r_foot_yaw = util.rot_to_rpy(rfoot_fin_iso[:3, :3])[2]
    if r_foot_yaw < 0:
        r_foot_yaw += 2 * np.pi
    # wrap around if around zero yaw crossing
    if np.abs(l_foot_yaw - r_foot_yaw) > np.pi:
        r_foot_yaw += 2 * np.pi
    base_yaw = (l_foot_yaw + r_foot_yaw) / 2.0
    base_fin_rot = util.euler_to_rot(np.array([0.0, 0.0, base_yaw]))
    base_fin_iso = liegroup.RpToTrans(base_fin_rot, base_fin_pos)

    return (
        swing_time,
        lfoot_ini_iso,
        lfoot_mid_iso,
        lfoot_fin_iso,
        lfoot_mid_vel,
        rfoot_ini_iso,
        rfoot_mid_iso,
        rfoot_fin_iso,
        rfoot_mid_vel,
        base_ini_iso,
        base_fin_iso,
    )


def create_curves(
    lfoot_ini_iso,
    lfoot_mid_iso,
    lfoot_fin_iso,
    lfoot_mid_vel,
    rfoot_ini_iso,
    rfoot_mid_iso,
    rfoot_fin_iso,
    rfoot_mid_vel,
    base_ini_iso,
    base_fin_iso,
):
    lfoot_pos_curve_ini_to_mid = interpolation.HermiteCurveVec(
        lfoot_ini_iso[0:3, 3], np.zeros(3), lfoot_mid_iso[0:3, 3], lfoot_mid_vel, 1.0
    )
    lfoot_pos_curve_mid_to_fin = interpolation.HermiteCurveVec(
        lfoot_mid_iso[0:3, 3], lfoot_mid_vel, lfoot_fin_iso[0:3, 3], np.zeros(3), 1.0
    )
    lfoot_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(lfoot_ini_iso[0:3, 0:3]),
        np.zeros(3),
        util.rot_to_quat(lfoot_fin_iso[0:3, 0:3]),
        np.zeros(3),
        1.0,
    )
    rfoot_pos_curve_ini_to_mid = interpolation.HermiteCurveVec(
        rfoot_ini_iso[0:3, 3], np.zeros(3), rfoot_mid_iso[0:3, 3], rfoot_mid_vel, 1.0
    )
    rfoot_pos_curve_mid_to_fin = interpolation.HermiteCurveVec(
        rfoot_mid_iso[0:3, 3], rfoot_mid_vel, rfoot_fin_iso[0:3, 3], np.zeros(3), 1.0
    )
    rfoot_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(rfoot_ini_iso[0:3, 0:3]),
        np.zeros(3),
        util.rot_to_quat(rfoot_fin_iso[0:3, 0:3]),
        np.zeros(3),
        1.0,
    )
    base_pos_curve = interpolation.HermiteCurveVec(
        base_ini_iso[0:3, 3], np.zeros(3), base_fin_iso[0:3, 3], np.zeros(3), 1.0
    )
    base_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(base_ini_iso[0:3, 0:3]),
        np.zeros(3),
        util.rot_to_quat(base_fin_iso[0:3, 0:3]),
        np.zeros(3),
        1.0,
    )

    return (
        lfoot_pos_curve_ini_to_mid,
        lfoot_pos_curve_mid_to_fin,
        lfoot_quat_curve,
        rfoot_pos_curve_ini_to_mid,
        rfoot_pos_curve_mid_to_fin,
        rfoot_quat_curve,
        base_pos_curve,
        base_quat_curve,
    )


def _do_generate_data(
    n_data,
    nominal_lf_iso,
    nominal_rf_iso,
    nominal_configuration,
    side,
    min_step_length=0.1,
    cw_or_ccw=None,
    rseed=None,
    cpu_idx=0,
    MAX_RESAMPLE_ATTEMPTS=50,
):
    # default to counter-clockwise if not specified
    if cw_or_ccw is None:
        cw_or_ccw = "ccw"

    if rseed is not None:
        np.random.seed(rseed)

    # initialize training / validation data size
    data_x = [
        np.zeros(
            NN_INPUT_DIM,
        )
        for _ in range(n_data)
    ]
    data_y = [
        np.zeros(
            NN_OUTPUT_DIM,
        )
        for _ in range(n_data)
    ]
    nominal_base_iso_init = configuration.get_transform_frame_to_world(
        "torso_link"
    ).copy()

    frame_idx = int(0)
    iter = 0
    text = "#" + "{}".format(cpu_idx).zfill(3)
    with tqdm(
        total=n_data, desc=text + ": Generating data", position=cpu_idx + 1
    ) as pbar:
        i = int(0)
        while i < n_data:
            # reset to home config
            configuration.q = copy.deepcopy(nominal_configuration)
            q_prev_step = copy.deepcopy(nominal_configuration)
            q_nominal = copy.deepcopy(nominal_configuration)
            # configuration.update()
            nominal_lf_iso_step = copy.deepcopy(nominal_lf_iso)
            nominal_rf_iso_step = copy.deepcopy(nominal_rf_iso)
            nominal_base_iso = copy.deepcopy(nominal_base_iso_init)
            n_turn = 0
            resample_attempt_num = int(0)
            while n_turn < N_TURN_STEPS:
                if MOTION_TYPE == MotionType.STEP:
                    (
                        swing_time,
                        lfoot_ini_iso,
                        lfoot_mid_iso,
                        lfoot_fin_iso,
                        lfoot_mid_vel,
                        rfoot_ini_iso,
                        rfoot_mid_iso,
                        rfoot_fin_iso,
                        rfoot_mid_vel,
                        base_ini_iso,
                        base_fin_iso,
                    ) = sample_swing_config(
                        nominal_lf_iso_step, nominal_rf_iso_step, side, min_step_length
                    )
                elif MOTION_TYPE == MotionType.TURN:
                    nominal_base_iso.translation[0] = (
                        nominal_lf_iso_step.translation[0]
                        + nominal_rf_iso_step.translation[0]
                    ) / 2.0
                    (
                        swing_time,
                        lfoot_ini_iso,
                        lfoot_mid_iso,
                        lfoot_fin_iso,
                        lfoot_mid_vel,
                        rfoot_ini_iso,
                        rfoot_mid_iso,
                        rfoot_fin_iso,
                        rfoot_mid_vel,
                        base_ini_iso,
                        base_fin_iso,
                    ) = sample_turn_config(
                        nominal_base_iso,
                        nominal_lf_iso_step,
                        nominal_rf_iso_step,
                        cw_or_ccw,
                        side,
                    )

                (
                    lfoot_pos_curve_ini_to_mid,
                    lfoot_pos_curve_mid_to_fin,
                    lfoot_quat_curve,
                    rfoot_pos_curve_ini_to_mid,
                    rfoot_pos_curve_mid_to_fin,
                    rfoot_quat_curve,
                    base_pos_curve,
                    base_quat_curve,
                ) = create_curves(
                    lfoot_ini_iso,
                    lfoot_mid_iso,
                    lfoot_fin_iso,
                    lfoot_mid_vel,
                    rfoot_ini_iso,
                    rfoot_mid_iso,
                    rfoot_fin_iso,
                    rfoot_mid_vel,
                    base_ini_iso,
                    base_fin_iso,
                )

                rate = RateLimiter(frequency=N_DATA_PER_MOTION / swing_time)
                dt = rate.period
                if VIDEO_RECORD:  # update frame rate
                    anim.default_framerate = int(1 / dt)

                s = 0.0
                b_terminate = False
                while s < 1.0:  # should generate N_DATA_PER_MOTION data points
                    base_pos = base_pos_curve.evaluate(s)
                    base_quat = base_quat_curve.evaluate(s)

                    if s <= 0.5:
                        sprime = 2.0 * s
                        lf_pos = lfoot_pos_curve_ini_to_mid.evaluate(sprime)
                        rf_pos = rfoot_pos_curve_ini_to_mid.evaluate(sprime)
                    else:
                        sprime = 2.0 * (s - 0.5)
                        lf_pos = lfoot_pos_curve_mid_to_fin.evaluate(sprime)
                        rf_pos = rfoot_pos_curve_mid_to_fin.evaluate(sprime)
                    lf_quat = lfoot_quat_curve.evaluate(s)
                    rf_quat = rfoot_quat_curve.evaluate(s)

                    # set desired task
                    des_base_iso = pin.SE3(util.quat_to_rot(base_quat), base_pos)
                    task_dict["torso_task"].set_target(des_base_iso)

                    des_rfoot_iso = pin.SE3(util.quat_to_rot(rf_quat), rf_pos)
                    task_dict["rfoot_task"].set_target(des_rfoot_iso)

                    des_lfoot_iso = pin.SE3(util.quat_to_rot(lf_quat), lf_pos)
                    task_dict["lfoot_task"].set_target(des_lfoot_iso)

                    task_dict["posture_task"].set_target(q_nominal)

                    # solve ik
                    # Compute velocity and integrate it into next configuration
                    velocity = solve_ik(configuration, tasks, dt, solver=solver)
                    configuration.integrate_inplace(velocity, dt)

                    # Compute all the collisions
                    pin.computeCollisions(
                        robot.model,
                        robot.data,
                        geom_model,
                        geom_data,
                        configuration.q,
                        False,
                    )

                    # Check for collisions at each time step
                    for k in range(len(geom_model.collisionPairs)):
                        cr = geom_data.collisionResults[k]
                        # cp = geom_model.collisionPairs[k]
                        if cr.isCollision():
                            # print("Collision between:",
                            # geom_model.geometryObjects[cp.first].name, ",",
                            # geom_model.geometryObjects[cp.second].name,
                            # ". Resampling step.")
                            resample_attempt_num += 1
                            b_terminate = True
                            break

                    # check that rolling contact joint constraint hasn't been violated
                    lk_cnstr_violated = (
                        np.abs(
                            configuration.q[NUM_FLOATING_BASE + lknee_jp_idx]
                            - configuration.q[NUM_FLOATING_BASE + lknee_jd_idx]
                        )
                        > MAX_HOL_ERROR
                    )
                    rk_cnstr_violated = (
                        np.abs(
                            configuration.q[NUM_FLOATING_BASE + rknee_jp_idx]
                            - configuration.q[NUM_FLOATING_BASE + rknee_jd_idx]
                        )
                        > MAX_HOL_ERROR
                    )
                    if lk_cnstr_violated or rk_cnstr_violated:
                        print(
                            "Rolling contact joint constraint violated. Resampling step."
                        )
                        b_terminate = True

                    if b_terminate:
                        break

                    # Visualize result at fixed FPS
                    if VISUALIZE:
                        viz.display(configuration.q)
                        # rate.sleep()

                        #### base
                        T_w_base = liegroup.RpToTrans(
                            util.quat_to_rot(base_quat), base_pos
                        )
                        #### right foot
                        T_w_rf = liegroup.RpToTrans(util.quat_to_rot(rf_quat), rf_pos)

                        #### left foot
                        T_w_lf = liegroup.RpToTrans(util.quat_to_rot(lf_quat), lf_pos)

                        lfoot_contact_frame.set_transform(T_w_lf)
                        rfoot_contact_frame.set_transform(T_w_rf)
                        torso_frame.set_transform(T_w_base)
                        if VIDEO_RECORD:
                            # update visualized frames
                            with anim.at_frame(lfoot_contact_frame, frame_idx) as frame:
                                frame.set_transform(T_w_lf)
                            with anim.at_frame(rfoot_contact_frame, frame_idx) as frame:
                                frame.set_transform(T_w_rf)
                            with anim.at_frame(torso_frame, frame_idx) as frame:
                                frame.set_transform(T_w_base)

                            # update robot configuration
                            with anim.at_frame(viz.viewer, frame_idx) as frame:
                                display_visualizer_frames(
                                    viz, frame, pin.GeometryType.VISUAL
                                )
                                display_visualizer_frames(
                                    viz, frame, pin.GeometryType.COLLISION
                                )

                    # update nominal configuration for next iteration
                    q_nominal = configuration.q
                    s += 1.0 / N_DATA_PER_MOTION
                    frame_idx += 1

                    # compute CRBI
                    _, _, centroidal_inertia = robot.centroidal(configuration.q, v0)
                    rot_inertia_in_world = np.copy(centroidal_inertia)[3:6, 3:6]

                    rot_w_base = util.quat_to_rot(base_quat)
                    rot_inertia_in_body = np.dot(
                        np.dot(rot_w_base.transpose(), rot_inertia_in_world), rot_w_base
                    )

                    # get rotations of feet w.r.t. base
                    base_rpy_lf = util.rot_to_rpy(
                        np.dot(rot_w_base.transpose(), util.quat_to_rot(lf_quat))
                    )
                    base_rpy_rf = util.rot_to_rpy(
                        np.dot(rot_w_base.transpose(), util.quat_to_rot(rf_quat))
                    )

                    # append data (end-effector SE3 & body inertia pair)
                    if i < n_data:
                        data_x[i] = np.concatenate(
                            [
                                lf_pos - base_pos,
                                rf_pos - base_pos,
                                base_rpy_lf,
                                base_rpy_rf,
                            ],
                            axis=0,
                        )
                        data_y[i] = np.array(
                            [
                                rot_inertia_in_body[0, 0],
                                rot_inertia_in_body[1, 1],
                                rot_inertia_in_body[2, 2],
                                rot_inertia_in_body[0, 1],
                                rot_inertia_in_body[0, 2],
                                rot_inertia_in_body[1, 2],
                            ]
                        )  # I_xx, I_yy, I_zz, I_xy, I_xz, I_yz
                    else:
                        print("")
                        print(f"Index {i} went out of bounds. Ignoring data point.")
                        break
                        # i = n_data - 2
                    i += 1

                # if in collision or violates RCJ, terminate and resample
                if b_terminate:
                    b_terminate = False
                    configuration.q = q_prev_step
                    q_nominal = q_prev_step
                    if resample_attempt_num < MAX_RESAMPLE_ATTEMPTS:
                        i = (
                            iter * N_DATA_PER_MOTION * N_TURN_STEPS
                            + n_turn * N_DATA_PER_MOTION
                        )
                    else:  # if really stuck
                        # and if the robot has taken 2 steps or more already
                        if n_turn > 1:
                            # restart from home pose and reset indices accordingly
                            configuration.q = copy.deepcopy(nominal_configuration)
                            configuration.update()
                            q_prev_step = copy.deepcopy(nominal_configuration)
                            q_nominal = copy.deepcopy(nominal_configuration)
                            nominal_lf_iso_step = copy.deepcopy(nominal_lf_iso)
                            nominal_rf_iso_step = copy.deepcopy(nominal_rf_iso)
                            nominal_base_iso = copy.deepcopy(nominal_base_iso_init)
                            n_turn = 0
                            resample_attempt_num = int(0)
                            print("")
                            print("Bad stuck. Re-starting from home config")
                            i = iter * N_DATA_PER_MOTION * N_TURN_STEPS
                    continue

                if MOTION_TYPE == MotionType.TURN:
                    # update nominal poses
                    nominal_lf_iso_step = pin.SE3(lfoot_fin_iso)
                    nominal_rf_iso_step = pin.SE3(rfoot_fin_iso)
                    nominal_base_iso = pin.SE3(base_fin_iso)
                    q_prev_step = configuration.q
                pbar.update(N_DATA_PER_MOTION)

                if MOTION_TYPE == MotionType.TURN:
                    # switch stance leg
                    if side == "right_leg":
                        side = "left_leg"
                    else:
                        side = "right_leg"

                # successful turning step without collision, continue to next step
                n_turn += 1

            # increase iteration count
            iter += 1
            if VISUALIZE:
                viz.display(configuration.q)
                # rate.sleep()

    return data_x, data_y


def _generate_data(arg_list):
    return _do_generate_data(*arg_list)


def generate_data(
    n_data,
    nominal_lf_iso,
    nominal_rf_iso,
    nominal_sensor_data,
    side,
    num_cpu,
    min_step_length=0.1,
    cw_or_ccw=None,
):
    if cw_or_ccw is None:
        cw_or_ccw = "ccw"

    rollout_per_cpu = int(max(n_data // num_cpu, 1))
    args_list = [
        rollout_per_cpu,
        nominal_lf_iso,
        nominal_rf_iso,
        nominal_sensor_data,
        side,
        min_step_length,
        cw_or_ccw,
    ]
    results = util.try_multiprocess(args_list, num_cpu, _generate_data)

    data_x, data_y = [], []
    for result in results:
        data_x += result[0]
        data_y += result[1]

    return data_x, data_y


def save_weights_to_yaml(pytorch_model, path):
    mlp_model = dict()
    mlp_model["num_layer"] = len(pytorch_model.layers)
    for l_id, lyr in enumerate(pytorch_model.layers):
        if hasattr(lyr, "weight"):
            mlp_model["w" + str(l_id)] = lyr.weight.tolist()
            mlp_model["b" + str(l_id)] = lyr.bias.reshape(
                1, lyr.weight.shape[0]
            ).tolist()
        else:  # is activation function
            # Activation Fn Idx: None: 0, Tanh: 1
            if l_id == (len(pytorch_model.layers) - 1):
                mlp_model["act_fn" + str(l_id)] = 0
            else:
                mlp_model["act_fn" + str(l_id)] = 1

    with open(path + "/mlp_model.yaml", "w") as f:
        yml = YAML()
        yml.dump(mlp_model, f)


def generate_casadi_func(
    pytorch_model, input_mean, input_std, output_mean, output_std, generate_c_code=True
):
    # c_code_path = cwd + "/data/pytorch_model/draco3_crbi"
    ## Computational Graph
    b_pos = MX.sym("b_pos", 3)
    l_pos = MX.sym("l_pos", 3)
    r_pos = MX.sym("r_pos", 3)
    b_ori = MX.sym("b_ori", 3)
    l_ori = MX.sym("l_ori", 3)
    r_ori = MX.sym("r_ori", 3)
    # Input
    l_pos_minus_b_pos = l_pos - b_pos
    r_pos_minus_b_pos = r_pos - b_pos
    l_ori_minus_b_ori = l_ori - b_ori
    r_ori_minus_b_ori = r_ori - b_ori
    inp = vertcat(
        l_pos_minus_b_pos, r_pos_minus_b_pos, l_ori_minus_b_ori, r_ori_minus_b_ori
    )
    normalized_inp = (inp - input_mean) / input_std  # (12, 1)
    # MLP (Somewhat manual)
    w0 = pytorch_model.layers[0].weight.detach().numpy()  # (12, 64)
    b0 = pytorch_model.layers[0].bias.detach().reshape(-1, 1).numpy()  # (12, 64)
    w1 = pytorch_model.layers[2].weight.detach().numpy()  # (64, 64)
    b1 = pytorch_model.layers[2].bias.detach().reshape(-1, 1).numpy()  # (1, 64)
    w2 = pytorch_model.layers[4].weight.detach().numpy()  # (64, 6)
    b2 = pytorch_model.layers[4].bias.detach().reshape(-1, 1).numpy()  # (6)

    output = (
        mtimes(w2, tanh(mtimes(w1, tanh(mtimes(w0, normalized_inp) + b0)) + b1)) + b2
    )
    denormalized_output = (output.T @ output_std) + output_mean

    # Define casadi function
    func = Function(
        "draco3_crbi_helper",
        [b_pos, b_ori, l_pos, l_ori, r_pos, r_ori],
        [denormalized_output],
    )
    jac_func = func.jacobian()
    print(func)
    print(jac_func)

    if generate_c_code:
        # Code generator
        code_gen = CodeGenerator("draco3_crbi_helper.c", dict(with_header=True))
        code_gen.add(func)
        code_gen.add(jac_func)
        code_gen.generate()
        shutil.move(
            cwd + "/draco3_crbi_helper.h", cwd + "/experiment_data/draco3_crbi_helper.h"
        )
        shutil.move(
            cwd + "/draco3_crbi_helper.c", cwd + "/experiment_data/draco3_crbi_helper.c"
        )

    return func, jac_func


def evaluate_crbi_model_using_casadi(cas_func, b, lft, r):
    out = cas_func(b, lft, r)
    return out


def evaluate_crbi_model_using_tf(
    tf_model, b, lft, r, input_mean, input_std, output_mean, output_std
):
    inp1 = lft - b
    inp2 = r - b
    inp = np.concatenate([inp1, inp2], axis=0)
    normalized_inp = np.expand_dims(util.normalize(inp, input_mean, input_std), axis=0)
    output = tf_model(normalized_inp)
    d_output = util.denormalize(np.squeeze(output), output_mean, output_std)
    return d_output, output


def load_turn_pink_config():
    # PInk costs optimized for turning motion
    torso_task.set_position_cost(10.0)
    torso_task.set_orientation_cost(5.0)
    posture_task.cost = 1e-3
    l_knee_holonomic_task.cost = 1e2
    l_knee_holonomic_task.gain = 0.05
    l_knee_holonomic_task.lm_damping = 5e-7
    r_knee_holonomic_task.cost = 1e2
    r_knee_holonomic_task.gain = 0.05
    r_knee_holonomic_task.lm_damping = 5e-7


def train_and_plot_crbi():
    log_dir = "experiment_data/tensorboard/draco3_crbi_por_ori"
    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    writer = SummaryWriter(log_dir)

    ## normalize training data
    print("{} data is collected".format(len(data_x)))
    input_mean, input_std, input_normalized_data = util.normalize_data(data_x)
    output_mean, output_std, output_normalized_data = util.normalize_data(data_y)

    # create Torch data set
    x_data_torch, y_data_torch = (
        torch.from_numpy(np.array(input_normalized_data)).float(),
        torch.from_numpy(np.array(output_normalized_data)).float(),
    )
    torch_dataset = torch_utils.TensorDataset(x_data_torch, y_data_torch)

    # Split into train and test data sets
    train_size = int(len(torch_dataset) * 0.9)
    test_size = len(torch_dataset) - train_size
    train_dataset, test_dataset = torch.utils.data.random_split(
        torch_dataset, [train_size, test_size]
    )

    # Load test / train data
    train_loader = torch_utils.DataLoader(
        dataset=train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=4
    )
    test_loader = torch_utils.DataLoader(
        dataset=test_dataset, batch_size=BATCH_SIZE, shuffle=False, num_workers=4
    )

    # Create NN model
    crbi_model = NetWork(
        NN_INPUT_DIM, N_LAYER_OUTPUT[0], N_LAYER_OUTPUT[1], NN_OUTPUT_DIM
    )
    optimizer = torch.optim.SGD(crbi_model.parameters(), lr=LR, momentum=0.0)
    loss_function = torch.nn.MSELoss()

    # train
    train_loss_per_epoch = 0.0
    # val_loss_per_epoch = 0.0
    iter = 0
    for epoch in range(N_EPOCH):
        current_loss = 0.0
        train_batch_loss = 0.0
        crbi_model.train()
        for step, (b_x, b_y) in enumerate(train_loader):
            optimizer.zero_grad()
            output = crbi_model(b_x)
            loss = loss_function(output, b_y)
            loss.backward()
            optimizer.step()
            train_batch_loss = loss.item()
            current_loss += loss.item()
            writer.add_scalar("Loss/Minibatches", train_batch_loss, iter)
            iter += 1
        train_loss_per_epoch = current_loss / len(train_loader)
        writer.add_scalar("Loss/Epochs", train_loss_per_epoch, epoch + 1)

    print("=" * 80)
    print("CRBI training done")

    # test
    crbi_model.eval()
    with torch.no_grad():
        test_loss_per_epoch = 0.0
        for epoch in range(N_EPOCH):
            test_loss = 0.0
            for i, (b_x, b_y) in enumerate(test_loader):
                output = crbi_model(b_x)
                loss = loss_function(output, b_y)
                test_loss += loss.item()
            test_loss_per_epoch = test_loss / len(test_loader)
            writer.add_scalar("TestLoss/Epochs", test_loss_per_epoch, epoch + 1)

    print("=" * 80)
    print("CRBI test done")

    #####################################################
    # save model
    #####################################################
    model_path = "experiment_data/pytorch_model/draco3_crbi_pos_ori.pth"
    if os.path.exists(model_path):
        os.remove(model_path)
    torch.save(crbi_model, model_path)
    print("==============================================")
    print("Saved PyTorch Model State")
    print("==============================================")

    data_stats = {
        "input_mean": input_mean.tolist(),
        "input_std": input_std.tolist(),
        "output_mean": output_mean.tolist(),
        "output_std": output_std.tolist(),
    }

    path = "experiment_data/pytorch_model/draco3_crbi"
    with open("experiment_data/pytorch_model" + "/data_stat.yaml", "w") as f:
        yml = YAML()
        yml.dump(data_stats, f)
    save_weights_to_yaml(crbi_model, path)
    print("=" * 80)
    print("Saved weights to yaml")

    cas_func, cas_jac_func = generate_casadi_func(
        crbi_model, input_mean, input_std, output_mean, output_std, True
    )

    #####################################################
    """plot centroidal inertia dim = 6"""
    #####################################################
    test_data_loader2 = torch_utils.DataLoader(
        dataset=test_dataset, batch_size=1, shuffle=False, num_workers=6
    )

    crbi_model.eval()
    gt_inertia_list, predict_inertia_list = [], []
    iter_list = []
    num_iter = 0
    with torch.no_grad():
        for i, (b_x, b_y) in enumerate(test_data_loader2):
            output = crbi_model(b_x)
            gt_denormalized_output = util.denormalize(
                np.squeeze(b_y.numpy(), axis=0), output_mean, output_std
            )
            predict_denormalized_output = util.denormalize(
                np.squeeze(output.numpy(), axis=0), output_mean, output_std
            )
            iter_list.append(num_iter)
            gt_inertia_list.append(gt_denormalized_output)
            predict_inertia_list.append(predict_denormalized_output)
            num_iter += 1

    ##plot
    fig, axes = plt.subplots(6, 1)
    for i in range(6):
        if i == 0:
            axes[i].plot(
                iter_list, [gt_inertia[i] for gt_inertia in gt_inertia_list], "r"
            )
            axes[i].plot(
                iter_list,
                [predict_inertia[i] for predict_inertia in predict_inertia_list],
                "b",
            )
            axes[i].grid(True)
        else:
            axes[i] = plt.subplot(6, 1, i + 1, sharex=axes[i - 1])
            axes[i].plot(
                iter_list, [gt_inertia[i] for gt_inertia in gt_inertia_list], "r"
            )
            axes[i].plot(
                iter_list,
                [predict_inertia[i] for predict_inertia in predict_inertia_list],
                "b",
            )
            axes[i].grid(True)
    plt.show()


if __name__ == "__main__":
    MOTION_TYPE = MotionType.STEP  # consider step by default
    NUM_FLOATING_BASE = 5  # pinocchio model

    # initialize robot model
    urdf_path = "/robot_model/draco/draco3_old.urdf"
    srdf_path = "/robot_model/draco/srdf/draco3_modified_collisions.srdf"
    robot = pin.RobotWrapper.BuildFromURDF(
        cwd + urdf_path,
        cwd + "/robot_model/draco",
        root_joint=pin.JointModelFreeFlyer(),
    )

    # load collision geometries
    geom_model = pin.buildGeomFromUrdf(
        robot.model,
        cwd + urdf_path,
        cwd + "/robot_model/draco",
        pin.GeometryType.COLLISION,
    )

    # add collision pairs and remove those listed in SRDF
    geom_model.addAllCollisionPairs()
    pin.removeCollisionPairs(robot.model, geom_model, cwd + srdf_path)

    # create data structure for collisions
    geom_data = pin.GeometryData(geom_model)

    # Initialize meschcat visualizer
    if VISUALIZE:
        viz = pin.visualize.MeshcatVisualizer(
            robot.model, geom_model, robot.visual_model
        )
        robot.setVisualizer(viz, init=False)
        viz.initViewer(open=True)
        viz.loadViewerModel()
        viz.display_collisions = True

    # Set initial robot configuration
    q0 = robot.q0.copy()
    q0[0:3] = np.array(INITIAL_POS_WORLD_TO_BASEJOINT)
    q0[3:7] = np.array(INITIAL_QUAT_WORLD_TO_BASEJOINT)
    lhip_aa_idx = robot.index("l_hip_aa")
    q0[NUM_FLOATING_BASE + lhip_aa_idx] = 0.04
    rhip_aa_idx = robot.index("r_hip_aa")
    q0[NUM_FLOATING_BASE + rhip_aa_idx] = -0.04
    lankle_ie_idx = robot.index("l_ankle_ie")
    q0[NUM_FLOATING_BASE + lankle_ie_idx] = -0.04
    rankle_ie_idx = robot.index("r_ankle_ie")
    q0[NUM_FLOATING_BASE + rankle_ie_idx] = 0.04
    lknee_jp_idx = robot.index("l_knee_fe_jp")
    q0[NUM_FLOATING_BASE + lknee_jp_idx] = np.pi / 4
    lknee_jd_idx = robot.index("l_knee_fe_jd")
    q0[NUM_FLOATING_BASE + lknee_jd_idx] = np.pi / 4
    rknee_jp_idx = robot.index("r_knee_fe_jp")
    q0[NUM_FLOATING_BASE + rknee_jp_idx] = np.pi / 4
    rknee_jd_idx = robot.index("r_knee_fe_jd")
    q0[NUM_FLOATING_BASE + rknee_jd_idx] = np.pi / 4
    l_hip_fe_idx = robot.index("l_hip_fe")
    q0[NUM_FLOATING_BASE + l_hip_fe_idx] = -np.pi / 4
    r_hip_fe_idx = robot.index("r_hip_fe")
    q0[NUM_FLOATING_BASE + r_hip_fe_idx] = -np.pi / 4
    l_ankle_fe_idx = robot.index("l_ankle_fe")
    q0[NUM_FLOATING_BASE + l_ankle_fe_idx] = -np.pi / 4
    r_ankle_fe_idx = robot.index("r_ankle_fe")
    q0[NUM_FLOATING_BASE + r_ankle_fe_idx] = -np.pi / 4
    l_shoulder_aa_idx = robot.index("l_shoulder_aa")
    q0[NUM_FLOATING_BASE + l_shoulder_aa_idx] = 0.523
    r_shoulder_aa_idx = robot.index("r_shoulder_aa")
    q0[NUM_FLOATING_BASE + r_shoulder_aa_idx] = -0.523
    l_elbow_fe_idx = robot.index("l_elbow_fe")
    q0[NUM_FLOATING_BASE + l_elbow_fe_idx] = -np.pi / 2
    r_elbow_fe_idx = robot.index("r_elbow_fe")
    q0[NUM_FLOATING_BASE + r_elbow_fe_idx] = -np.pi / 2
    ## avoid joint violation
    configuration = pink.Configuration(robot.model, robot.data, q0)
    v0 = np.zeros(len(q0) - 1)

    if VISUALIZE:
        viz.display(configuration.q)

    # Tasks initialization for IK
    left_foot_task = FrameTask(
        "l_foot_contact",
        position_cost=10.0,
        orientation_cost=1.0,
    )
    torso_task = FrameTask(
        "torso_link",
        position_cost=1.0,
        orientation_cost=1.0,
    )
    right_foot_task = FrameTask(
        "r_foot_contact",
        position_cost=10.0,
        orientation_cost=1.0,
    )
    posture_task = PostureTask(
        cost=1e-3,  # [cost] / [rad]
    )

    # Joint coupling task
    r_knee_holonomic_task = JointCouplingTask(
        ["r_knee_fe_jp", "r_knee_fe_jd"],
        [1.0, -1.0],
        500.0,
        configuration,
        lm_damping=1e-7,
    )
    r_knee_holonomic_task.gain = 0.1
    l_knee_holonomic_task = JointCouplingTask(
        ["l_knee_fe_jp", "l_knee_fe_jd"],
        [1.0, -1.0],
        500.0,
        configuration,
        lm_damping=1e-7,
    )
    l_knee_holonomic_task.gain = 0.1

    tasks = [
        left_foot_task,
        torso_task,
        right_foot_task,
        posture_task,
        l_knee_holonomic_task,
        r_knee_holonomic_task,
    ]
    task_dict = {
        "torso_task": torso_task,
        "lfoot_task": left_foot_task,
        "rfoot_task": right_foot_task,
        "posture_task": posture_task,
        "lknee_constr_task": l_knee_holonomic_task,
        "rknee_constr_task": r_knee_holonomic_task,
    }

    nominal_base_iso = configuration.get_transform_frame_to_world("torso_link").copy()
    nominal_rf_iso = configuration.get_transform_frame_to_world("r_foot_contact").copy()
    nominal_lf_iso = configuration.get_transform_frame_to_world("l_foot_contact").copy()

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"

    # meshcat visualizer
    if VISUALIZE:
        rfoot_contact_frame = viz.viewer["r_foot_contact"]
        meshcat_shapes.frame(rfoot_contact_frame)

        lfoot_contact_frame = viz.viewer["l_foot_contact"]
        meshcat_shapes.frame(lfoot_contact_frame)

        torso_frame = viz.viewer["torso_link"]
        meshcat_shapes.frame(torso_frame)

    # =========================================================================
    # Options:
    # Case 0: Train CRBI Regressor w/multiprocessing
    # Case 1: Load Pre-trained CRBI Model
    # Case 2: Sample Motions - straight footsteps (left/right side)
    # Case 3: Sample Motions - long CCW turns
    # Case 4: Train CRBI Regressor w/multiprocessing for long CCW turns
    # Case 5: Train CRBI Regressor w/multiprocessing for forward walking + long CCW turns
    # =========================================================================
    CASE = 5

    # Set base_pos, base_quat, joint_pos here for visualization
    if CASE == 0:
        # Generate Dataset
        print("-" * 80)
        print("Case 0: Train CRBI Regressor w/multiprocessing")
        VISUALIZE = False  # can't do this with multiprocessing
        VIDEO_RECORD = False
        min_step_length = 0.0

        FOOT_EA_LB = np.array([np.deg2rad(0.0), np.deg2rad(0.0), 0.0])
        FOOT_EA_UB = np.array([np.deg2rad(0.0), np.deg2rad(0.0), 0.0])

        RFOOT_POS_LB = np.array([0.0, -0.1, -0.05])
        RFOOT_POS_UB = np.array([0.5, 0.05, 0.05])
        LFOOT_POS_LB = np.array([0.0, -0.05, -0.05])
        LFOOT_POS_UB = np.array([0.5, 0.1, 0.05])
        min_step_length = 0.5
        N_TURN_STEPS = 1  # added to comply with turning option

        nominal_configuration = q0
        l_data_x, l_data_y = generate_data(
            N_MOTION_PER_LEG,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "left",
            N_CPU_DATA_GEN,
            min_step_length,
        )
        r_data_x, r_data_y = generate_data(
            N_MOTION_PER_LEG,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "right",
            N_CPU_DATA_GEN,
            min_step_length,
        )
        data_x = l_data_x + r_data_x
        data_y = l_data_y + r_data_y

        train_and_plot_crbi()

        exit(0)

    elif CASE == 1:
        print("-" * 80)
        print("Case 1: Load Pre-trained CRBI Model")
        crbi_model = torch.load("experiment_data/pytorch_model/draco3_crbi_pos_ori.pth")
        model_path = "experiment_data/pytorch_model/draco3_crbi"
        with open(model_path + "/data_stat.yaml", "r") as f:
            yml = YAML().load(f)
            input_mean = np.array(yml["input_mean"])
            input_std = np.array(yml["input_std"])
            output_mean = np.array(yml["output_mean"])
            output_std = np.array(yml["output_std"])

        b_regressor_trained = True

    elif CASE == 2:
        # Left Foot Swing, Right Foot Stance
        print("-" * 80)
        print("Case 2: Sample Motions - straight footsteps (left/right side)")

        FOOT_EA_LB = np.array([np.deg2rad(0.0), np.deg2rad(0.0), 0.0])
        FOOT_EA_UB = np.array([np.deg2rad(0.0), np.deg2rad(0.0), 0.0])

        RFOOT_POS_LB = np.array([0.0, -0.1, -0.05])
        RFOOT_POS_UB = np.array([0.5, 0.05, 0.05])
        LFOOT_POS_LB = np.array([0.0, -0.05, -0.05])
        LFOOT_POS_UB = np.array([0.5, 0.1, 0.05])
        N_TURN_STEPS = 1  # single step motion, repeated several times

        if VIDEO_RECORD:
            anim = meshcat.animation.Animation()
        _do_generate_data(
            N_DATA_PER_MOTION * N_TURN_ITERS,
            nominal_lf_iso,
            nominal_rf_iso,
            q0,
            "right",
        )

        if VIDEO_RECORD:
            viz.viewer.set_animation(anim, play=False)

    elif CASE == 3:
        # Left Foot Stance, Right Foot Swing
        print("-" * 80)
        print("Case 3: Sample Motions - long CCW turns")
        q_prev_step = q0.copy()

        # place base above ankles
        nominal_base_iso.translation[0] = (
            nominal_lf_iso.translation[0] + nominal_rf_iso.translation[0]
        ) / 2.0

        load_turn_pink_config()
        MOTION_TYPE = MotionType.TURN

        # Turning only in yaw
        FOOT_EA_LB = np.array([np.deg2rad(-0.1), np.deg2rad(-0.1), -np.pi / 2.0])
        FOOT_EA_UB = np.array([np.deg2rad(0.1), np.deg2rad(0.1), np.pi / 2.0])

        # Reduce stepping outwards (relative to stance leg, in stance coordinates)
        TURN_SWING_POS_LB = np.array([0.0, -0.4, 0.0])
        TURN_SWING_POS_UB = np.array([0.3, -0.2, 0.0])
        TURN_STANCE_POS_LB = np.array([-0.15, 0.15, 0.0])
        TURN_STANCE_POS_UB = np.array([0.15, 0.3, 0.0])

        # no need to lift leg as high
        SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.05, 0.15
        SWING_TIME_LB, SWING_TIME_UB = 1.5, 1.8
        BASE_HEIGHT_LB, BASE_HEIGHT_UB = (
            configuration.q[2] - 0.02,
            configuration.q[2] + 0.02,
        )

        if VIDEO_RECORD:
            anim = meshcat.animation.Animation()

        num_iters = 3  # number of times the turning motion is repeated
        first_swing_leg = "right_leg"
        _do_generate_data(
            N_DATA_PER_MOTION * N_TURN_STEPS * num_iters,
            nominal_lf_iso,
            nominal_rf_iso,
            q0,
            first_swing_leg,
            cw_or_ccw="ccw",
        )

        if VIDEO_RECORD:
            viz.viewer.set_animation(anim, play=False)

    elif CASE == 4:
        # CCW turning, right foot first
        print("-" * 80)
        print("Case 4: Train CRBI Regressor w/multiprocessing for long CCW turns")
        VISUALIZE = False  # can't do this with multiprocessing
        VIDEO_RECORD = False

        # place base above ankles
        nominal_base_iso.translation[0] = (
            nominal_lf_iso.translation[0] + nominal_rf_iso.translation[0]
        ) / 2.0

        load_turn_pink_config()
        MOTION_TYPE = MotionType.TURN

        # Turning only in yaw
        FOOT_EA_LB = np.array([np.deg2rad(-0.1), np.deg2rad(-0.1), -np.pi / 2.0])
        FOOT_EA_UB = np.array([np.deg2rad(0.1), np.deg2rad(0.1), np.pi / 2.0])

        # Reduce stepping outwards (relative to stance leg, in stance coordinates)
        TURN_SWING_POS_LB = np.array([0.0, -0.4, 0.0])
        TURN_SWING_POS_UB = np.array([0.3, -0.2, 0.0])
        TURN_STANCE_POS_LB = np.array([-0.15, 0.15, 0.0])
        TURN_STANCE_POS_UB = np.array([0.15, 0.3, 0.0])

        # no need to lift leg as high
        SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.05, 0.15
        SWING_TIME_LB, SWING_TIME_UB = 1.5, 1.8
        BASE_HEIGHT_LB, BASE_HEIGHT_UB = (
            configuration.q[2] - 0.02,
            configuration.q[2] + 0.02,
        )

        if VIDEO_RECORD:
            anim = meshcat.animation.Animation()

        nominal_configuration = q0
        l_data_x, l_data_y = generate_data(
            N_DATA_PER_MOTION * N_TURN_STEPS * N_TURN_ITERS,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "left_leg",
            N_CPU_DATA_GEN,
            cw_or_ccw="cw",
        )

        r_data_x, r_data_y = generate_data(
            N_DATA_PER_MOTION * N_TURN_STEPS * N_TURN_ITERS,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "right_leg",
            N_CPU_DATA_GEN,
            cw_or_ccw="ccw",
        )
        data_x = l_data_x + r_data_x
        data_y = l_data_y + r_data_y

        print("-------------------------------------------")
        print("data_x len: ", len(data_x))
        print("-------------------------------------------")

        train_and_plot_crbi()

        if VIDEO_RECORD:
            viz.viewer.set_animation(anim, play=False)

    elif CASE == 5:
        # CCW turning, right foot first
        print("-" * 80)
        print(
            "Case 5: Train CRBI Regressor w/multiprocessing for forward walking + long CCW turns"
        )
        VISUALIZE = False  # can't do this with multiprocessing
        VIDEO_RECORD = False

        nominal_configuration = q0

        ## =======================================================================
        ## forward motions
        ## =======================================================================
        MOTION_TYPE = MotionType.STEP
        N_DATA_PER_MOTION = 30
        N_SWING_MOTIONS = 4e3
        N_TURN_STEPS = 1  # single step motion, repeated several times

        SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.05, 0.30
        SWING_TIME_LB, SWING_TIME_UB = 0.35, 0.75
        BASE_HEIGHT_LB, BASE_HEIGHT_UB = 0.86, 0.95

        FOOT_EA_LB = np.array([np.deg2rad(-5.0), np.deg2rad(-15.0), -np.pi / 3.0])
        FOOT_EA_UB = np.array([np.deg2rad(5.0), np.deg2rad(15.0), np.pi / 3.0])
        # FOOT_EA_LB = np.array([np.deg2rad(0.), np.deg2rad(0.), 0.])
        # FOOT_EA_UB = np.array([np.deg2rad(0.), np.deg2rad(0.), 0.])

        RFOOT_POS_LB = np.array([0.0, -0.1, -0.05])
        RFOOT_POS_UB = np.array([0.5, 0.05, 0.05])
        LFOOT_POS_LB = np.array([0.0, -0.05, -0.05])
        LFOOT_POS_UB = np.array([0.5, 0.1, 0.05])

        l_swing_data_x, l_swing_data_y = generate_data(
            N_DATA_PER_MOTION * N_SWING_MOTIONS,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "left",
            N_CPU_DATA_GEN,
        )
        r_swing_data_x, r_swing_data_y = generate_data(
            N_DATA_PER_MOTION * N_SWING_MOTIONS,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "right",
            N_CPU_DATA_GEN,
        )

        ## =======================================================================
        ## turning motions
        ## =======================================================================
        MOTION_TYPE = MotionType.TURN
        N_DATA_PER_MOTION = 30
        N_TURN_STEPS = 4
        N_TURN_ITERS = 1e3  # number of iterations of each turning sequence

        # place base above ankles
        nominal_base_iso.translation[0] = (
            nominal_lf_iso.translation[0] + nominal_rf_iso.translation[0]
        ) / 2.0

        load_turn_pink_config()

        # Turning only in yaw
        FOOT_EA_LB = np.array([np.deg2rad(-0.1), np.deg2rad(-0.1), -np.pi / 2.0])
        FOOT_EA_UB = np.array([np.deg2rad(0.1), np.deg2rad(0.1), np.pi / 2.0])

        # Reduce stepping outwards (relative to stance leg, in stance coordinates)
        TURN_SWING_POS_LB = np.array([0.0, -0.4, 0.0])
        TURN_SWING_POS_UB = np.array([0.3, -0.2, 0.0])
        TURN_STANCE_POS_LB = np.array([-0.15, 0.15, 0.0])
        TURN_STANCE_POS_UB = np.array([0.15, 0.3, 0.0])

        # no need to lift leg as high
        SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.05, 0.30
        SWING_TIME_LB, SWING_TIME_UB = 1.5, 1.8
        # BASE_HEIGHT_LB, BASE_HEIGHT_UB = configuration.q[2]-0.02, configuration.q[2]+0.02
        BASE_HEIGHT_LB, BASE_HEIGHT_UB = 0.86, 0.95

        l_turn_data_x, l_turn_data_y = generate_data(
            N_DATA_PER_MOTION * N_TURN_STEPS * N_TURN_ITERS,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "left_leg",
            N_CPU_DATA_GEN,
            cw_or_ccw="cw",
        )

        r_turn_data_x, r_turn_data_y = generate_data(
            N_DATA_PER_MOTION * N_TURN_STEPS * N_TURN_ITERS,
            nominal_lf_iso,
            nominal_rf_iso,
            nominal_configuration,
            "right_leg",
            N_CPU_DATA_GEN,
            cw_or_ccw="ccw",
        )
        ## =======================================================================
        ## train
        ## =======================================================================

        data_x = l_swing_data_x + r_swing_data_x + l_turn_data_x + r_turn_data_x
        data_y = l_swing_data_y + r_swing_data_y + l_turn_data_y + r_turn_data_y
        # data_x = l_swing_data_x + r_swing_data_x
        # data_y = l_swing_data_y + r_swing_data_y

        print("-------------------------------------------")
        print("data_x len: ", len(data_x))
        print("-------------------------------------------")

        train_and_plot_crbi()

    if SAVE_DATA:
        data_saver.close()
