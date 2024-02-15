import os
import sys

import meshcat

cwd = os.getcwd()
sys.path.append(cwd)
import time
import math
import copy
import shutil
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=3)

from tqdm import tqdm
from ruamel.yaml import YAML
from casadi import *

from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import interpolation
from util.python_utils import liegroup

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


## Configs
VISUALIZE = True
VIDEO_RECORD = True
PRINT_FREQ = 10
DT = 0.01
PRINT_ROBOT_INFO = False
INITIAL_POS_WORLD_TO_BASEJOINT = [0, 0, 1.5 - 0.761]
INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0., 1.]

## Motion Boundaries
RFOOT_POS_LB = np.array([-0.15, -0.1, -0.05])
RFOOT_POS_UB = np.array([0.15, 0.05, 0.05])
LFOOT_POS_LB = np.array([-0.15, -0.05, -0.05])
LFOOT_POS_UB = np.array([0.15, 0.1, 0.05])

FOOT_EA_LB = np.array([np.deg2rad(-5.), np.deg2rad(-15.), -np.pi / 4.])
FOOT_EA_UB = np.array([np.deg2rad(5.), np.deg2rad(15.), np.pi / 4.])

SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.03, 0.20

SWING_TIME_LB, SWING_TIME_UB = 0.35, 0.75

BASE_HEIGHT_LB, BASE_HEIGHT_UB = 0.7, 0.8

## Dataset Generation
N_CPU_DATA_GEN = 5
N_MOTION_PER_LEG = 1e3
N_DATA_PER_MOTION = 15

N_LAYER_OUTPUT = [64, 64]
LR = 0.01
MOMENTUM = 0.
N_EPOCH = 20
BATCH_SIZE = 32


## 2-hidden layer Neural Network
class NetWork(torch.nn.Module):

    def __init__(self, n_input, n_hidden_1, n_hidden_2, n_output):
        super(NetWork, self).__init__()
        self.layers = torch.nn.Sequential(
            torch.nn.Linear(n_input, n_hidden_1), torch.nn.Tanh(),
            torch.nn.Linear(n_hidden_1, n_hidden_2), torch.nn.Tanh(),
            torch.nn.Linear(n_hidden_2, n_output))

    def forward(self, x):
        return self.layers(x)


def display_visualizer_frames(meshcat_visualizer, frame):
    for visual in meshcat_visualizer.visual_model.geometryObjects:
        # Get mesh pose.
        M = meshcat_visualizer.visual_data.oMg[
            meshcat_visualizer.visual_model.getGeometryId(visual.name)]
        # Manage scaling
        scale = np.asarray(visual.meshScale).flatten()
        S = np.diag(np.concatenate((scale, [1.0])))
        T = np.array(M.homogeneous).dot(S)
        # Update viewer configuration.
        frame[meshcat_visualizer.getViewerNodeName(
            visual, pin.GeometryType.VISUAL)].set_transform(T)


def sample_swing_config(nominal_lf_iso, nominal_rf_iso, side, min_step_length=1.):

    swing_time = np.random.uniform(SWING_TIME_LB, SWING_TIME_UB)
    min_y_distance_btw_foot = abs(nominal_lf_iso.translation[1] - nominal_rf_iso.translation[1])
    # TODO add swing height
    if side == "left":
        # Sample rfoot config
        rfoot_ini_iso = np.copy(nominal_rf_iso)
        rfoot_mid_iso = np.copy(nominal_rf_iso)
        rfoot_fin_iso = np.copy(nominal_rf_iso)

        rfoot_mid_vel = (rfoot_ini_iso[0:3, 3] -
                         rfoot_ini_iso[0:3, 3]) / swing_time
        rfoot_mid_vel[2] = 0.

        # Sample lfoot config
        lfoot_ini_pos = np.copy(nominal_lf_iso)[0:3, 3] + np.random.uniform(
            LFOOT_POS_LB, LFOOT_POS_UB)
        lfoot_ini_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        lfoot_ini_rot = util.euler_to_rot(lfoot_ini_ea)
        lfoot_ini_iso = liegroup.RpToTrans(lfoot_ini_rot, lfoot_ini_pos)
        lfoot_fin_pos = np.copy(nominal_lf_iso)[0:3, 3] + np.random.uniform(
            LFOOT_POS_LB, LFOOT_POS_UB)

        # resample until min distance in y-direction is satisfied
        while (abs(lfoot_ini_pos[1] - nominal_rf_iso.translation[1]) <
               min_y_distance_btw_foot):
            lfoot_ini_pos = nominal_lf_iso.translation + np.random.uniform(
                LFOOT_POS_LB, LFOOT_POS_UB)
        while (abs(lfoot_fin_pos[1] - nominal_rf_iso.translation[1]) <
               min_y_distance_btw_foot):
            lfoot_fin_pos = nominal_lf_iso.translation + np.random.uniform(
                LFOOT_POS_LB, LFOOT_POS_UB)

        # modify x-direction sample until min distance in x-direction is satisfied
        while (abs(lfoot_fin_pos[0] - nominal_rf_iso.translation[0]) <
               min_step_length):
            lfoot_fin_pos[0] = lfoot_fin_pos[0] + np.random.uniform(
                0., LFOOT_POS_UB[0])

        lfoot_fin_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        lfoot_fin_rot = util.euler_to_rot(lfoot_fin_ea)
        lfoot_fin_iso = liegroup.RpToTrans(lfoot_fin_rot, lfoot_fin_pos)
        lfoot_mid_iso = interpolation.iso_interpolate(lfoot_ini_iso,
                                                      lfoot_fin_iso, 0.5)
        lfoot_mid_iso[2, 3] = (lfoot_ini_pos[2] +
                               lfoot_fin_pos[2]) / 2.0 + np.random.uniform(
            SWING_HEIGHT_LB, SWING_HEIGHT_UB)

        lfoot_mid_vel = (lfoot_fin_pos - lfoot_ini_pos) / swing_time
        lfoot_mid_vel[2] = 0.

        # Sample base config
        base_ini_pos = (rfoot_ini_iso[0:3, 3] + lfoot_ini_iso[0:3, 3]) / 2.0
        base_ini_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_ini_rot = util.euler_to_rot(
            np.array([0., 0., lfoot_ini_ea[2] / 2.]))
        base_ini_iso = liegroup.RpToTrans(base_ini_rot, base_ini_pos)
        base_fin_pos = (rfoot_fin_iso[0:3, 3] + lfoot_fin_iso[0:3, 3]) / 2.0
        base_fin_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_fin_rot = util.euler_to_rot(
            np.array([0., 0., lfoot_fin_ea[2] / 2.]))
        base_fin_iso = liegroup.RpToTrans(base_fin_rot, base_fin_pos)

    elif side == "right":
        # Sample lfoot config
        lfoot_ini_iso = np.copy(nominal_lf_iso)
        lfoot_mid_iso = np.copy(nominal_lf_iso)
        lfoot_fin_iso = np.copy(nominal_lf_iso)

        lfoot_mid_vel = (lfoot_ini_iso[0:3, 3] -
                         lfoot_ini_iso[0:3, 3]) / swing_time
        lfoot_mid_vel[2] = 0.

        # Sample rfoot config
        rfoot_ini_pos = np.copy(nominal_rf_iso)[0:3, 3] + np.random.uniform(
            RFOOT_POS_LB, RFOOT_POS_UB)
        rfoot_ini_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        rfoot_ini_rot = util.euler_to_rot(rfoot_ini_ea)
        rfoot_ini_iso = liegroup.RpToTrans(rfoot_ini_rot, rfoot_ini_pos)
        rfoot_fin_pos = np.copy(nominal_rf_iso)[0:3, 3] + np.random.uniform(
            RFOOT_POS_LB, RFOOT_POS_UB)

        # resample until min distance in y-direction is satisfied
        while (abs(rfoot_ini_pos[1] - nominal_lf_iso.translation[1]) <
               min_y_distance_btw_foot):
            rfoot_ini_pos = nominal_rf_iso.translation + np.random.uniform(
                RFOOT_POS_LB, RFOOT_POS_UB)
        while (abs(rfoot_fin_pos[1] - nominal_lf_iso.translation[1]) <
               min_y_distance_btw_foot):
            rfoot_fin_pos = nominal_rf_iso.translation + np.random.uniform(
                RFOOT_POS_LB, RFOOT_POS_UB)

        # modify x-direction sample until min distance in x-direction is satisfied
        while (abs(rfoot_fin_pos[0] - nominal_lf_iso.translation[0]) <
               min_step_length):
            rfoot_fin_pos[0] = rfoot_fin_pos[0] + np.random.uniform(
                0., RFOOT_POS_UB[0])

        rfoot_fin_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        rfoot_fin_rot = util.euler_to_rot(rfoot_fin_ea)
        rfoot_fin_iso = liegroup.RpToTrans(rfoot_fin_rot, rfoot_fin_pos)
        rfoot_mid_iso = interpolation.iso_interpolate(rfoot_ini_iso,
                                                      rfoot_fin_iso, 0.5)
        rfoot_mid_iso[2, 3] = (rfoot_ini_pos[2] +
                               rfoot_fin_pos[2]) / 2.0 + np.random.uniform(
            SWING_HEIGHT_LB, SWING_HEIGHT_UB)

        rfoot_mid_vel = (rfoot_fin_pos - rfoot_ini_pos) / swing_time
        rfoot_mid_vel[2] = 0.

        # Sample base config
        base_ini_pos = (rfoot_ini_iso[0:3, 3] + lfoot_ini_iso[0:3, 3]) / 2.0
        base_ini_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_ini_rot = util.euler_to_rot(
            np.array([0., 0., rfoot_ini_ea[2] / 2.]))
        base_ini_iso = liegroup.RpToTrans(base_ini_rot, base_ini_pos)
        base_fin_pos = (rfoot_fin_iso[0:3, 3] + lfoot_fin_iso[0:3, 3]) / 2.0
        base_fin_pos[2] = np.random.uniform(BASE_HEIGHT_LB, BASE_HEIGHT_UB)
        base_fin_rot = util.euler_to_rot(
            np.array([0., 0., rfoot_fin_ea[2] / 2.]))
        base_fin_iso = liegroup.RpToTrans(base_fin_rot, base_fin_pos)

    else:
        raise ValueError

    return swing_time, lfoot_ini_iso, lfoot_mid_iso, lfoot_fin_iso, lfoot_mid_vel, rfoot_ini_iso, rfoot_mid_iso, rfoot_fin_iso, rfoot_mid_vel, base_ini_iso, base_fin_iso


def create_curves(lfoot_ini_iso, lfoot_mid_iso, lfoot_fin_iso, lfoot_mid_vel,
                  rfoot_ini_iso, rfoot_mid_iso, rfoot_fin_iso, rfoot_mid_vel,
                  base_ini_iso, base_fin_iso):
    lfoot_pos_curve_ini_to_mid = interpolation.HermiteCurveVec(
        lfoot_ini_iso[0:3, 3], np.zeros(3), lfoot_mid_iso[0:3, 3],
        lfoot_mid_vel, 1.)
    lfoot_pos_curve_mid_to_fin = interpolation.HermiteCurveVec(
        lfoot_mid_iso[0:3, 3], lfoot_mid_vel, lfoot_fin_iso[0:3, 3],
        np.zeros(3), 1.)
    lfoot_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(lfoot_ini_iso[0:3, 0:3]), np.zeros(3),
        util.rot_to_quat(lfoot_fin_iso[0:3, 0:3]), np.zeros(3), 1.)
    rfoot_pos_curve_ini_to_mid = interpolation.HermiteCurveVec(
        rfoot_ini_iso[0:3, 3], np.zeros(3), rfoot_mid_iso[0:3, 3],
        rfoot_mid_vel, 1.)
    rfoot_pos_curve_mid_to_fin = interpolation.HermiteCurveVec(
        rfoot_mid_iso[0:3, 3], rfoot_mid_vel, rfoot_fin_iso[0:3, 3],
        np.zeros(3), 1.)
    rfoot_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(rfoot_ini_iso[0:3, 0:3]), np.zeros(3),
        util.rot_to_quat(rfoot_fin_iso[0:3, 0:3]), np.zeros(3), 1.)
    base_pos_curve = interpolation.HermiteCurveVec(base_ini_iso[0:3, 3],
                                                   np.zeros(3),
                                                   base_fin_iso[0:3, 3],
                                                   np.zeros(3), 1.)
    base_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(base_ini_iso[0:3, 0:3]), np.zeros(3),
        util.rot_to_quat(base_fin_iso[0:3, 0:3]), np.zeros(3), 1.)

    return lfoot_pos_curve_ini_to_mid, lfoot_pos_curve_mid_to_fin, lfoot_quat_curve, rfoot_pos_curve_ini_to_mid, rfoot_pos_curve_mid_to_fin, rfoot_quat_curve, base_pos_curve, base_quat_curve


def _do_generate_data(n_data,
                      nominal_lf_iso,
                      nominal_rf_iso,
                      nominal_configuration,
                      side,
                      min_step_length=1.,
                      rseed=None,
                      cpu_idx=0):
    if rseed is not None:
        np.random.seed(rseed)
    data_x, data_y = [], []

    frame_idx = int(0)
    text = "#" + "{}".format(cpu_idx).zfill(3)
    with tqdm(total=n_data,
              desc=text + ': Generating data',
              position=cpu_idx + 1) as pbar:
        for i in range(n_data):

            swing_time, lfoot_ini_iso, lfoot_mid_iso, lfoot_fin_iso, lfoot_mid_vel, rfoot_ini_iso, rfoot_mid_iso, rfoot_fin_iso, rfoot_mid_vel, base_ini_iso, base_fin_iso = sample_swing_config(
                nominal_lf_iso, nominal_rf_iso, side, min_step_length)

            lfoot_pos_curve_ini_to_mid, lfoot_pos_curve_mid_to_fin, lfoot_quat_curve, rfoot_pos_curve_ini_to_mid, rfoot_pos_curve_mid_to_fin, rfoot_quat_curve, base_pos_curve, base_quat_curve = create_curves(
                lfoot_ini_iso, lfoot_mid_iso, lfoot_fin_iso, lfoot_mid_vel,
                rfoot_ini_iso, rfoot_mid_iso, rfoot_fin_iso, rfoot_mid_vel,
                base_ini_iso, base_fin_iso)

            t = 0.
            rate = RateLimiter(frequency=N_DATA_PER_MOTION / swing_time)
            dt = rate.period
            for s in np.linspace(0, 1, N_DATA_PER_MOTION):
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
                task_dict['torso_task'].set_target(des_base_iso)

                des_rfoot_iso = pin.SE3(util.quat_to_rot(rf_quat), rf_pos)
                task_dict['rfoot_task'].set_target(des_rfoot_iso)

                des_lfoot_iso = pin.SE3(util.quat_to_rot(lf_quat), lf_pos)
                task_dict['lfoot_task'].set_target(des_lfoot_iso)

                #TODO: or set from configuration
                task_dict['posture_task'].set_target(nominal_configuration)

                # update meshcat visualizer
                if VISUALIZE:
                    #### base
                    T_w_base = liegroup.RpToTrans(util.quat_to_rot(base_quat),
                                                  base_pos)
                    #### right foot
                    T_w_rf = liegroup.RpToTrans(util.quat_to_rot(rf_quat), rf_pos)

                    #### left foot
                    T_w_lf = liegroup.RpToTrans(util.quat_to_rot(lf_quat), lf_pos)

                    lfoot_contact_frame.set_transform(T_w_lf)
                    rfoot_contact_frame.set_transform(T_w_rf)
                    if VIDEO_RECORD:
                        with anim.at_frame(lfoot_contact_frame, frame_idx) as frame:
                            frame.set_transform(T_w_lf)
                        with anim.at_frame(rfoot_contact_frame, frame_idx) as frame:
                            frame.set_transform(T_w_rf)

                # solve ik
                # Compute velocity and integrate it into next configuration
                velocity = solve_ik(configuration, tasks, dt, solver=solver)
                configuration.integrate_inplace(velocity, dt)

                # Visualize result at fixed FPS
                if VISUALIZE:
                    viz.display(configuration.q)
                    if VIDEO_RECORD:
                        with anim.at_frame(viz.viewer, frame_idx) as frame:
                            display_visualizer_frames(viz, frame)
                    rate.sleep()
                t += dt
                frame_idx += 1

                # compute CRBI
                _, _, centroidal_inertia = robot.centroidal(configuration.q, v0)
                rot_inertia_in_world = np.copy(centroidal_inertia)[3:6, 3:6]

                rot_w_base = util.quat_to_rot(base_quat)
                rot_inertia_in_body = np.dot(np.dot(rot_w_base.transpose(), rot_inertia_in_world), rot_w_base)

                # append data (end-effector SE3 & body inertia pair)
                data_x.append(
                    np.concatenate([
                        lf_pos - base_pos, rf_pos - base_pos,
                        ],
                        axis=0))
                data_y.append(
                    np.array([
                        rot_inertia_in_body[0, 0], rot_inertia_in_body[1, 1], rot_inertia_in_body[2, 2], rot_inertia_in_body[0, 1],
                        rot_inertia_in_body[0, 2], rot_inertia_in_body[1, 2]
                    ]))
                pbar.update(1)

            # reset config
            configuration.q = q0
            configuration.update()
            if VISUALIZE:
                viz.display(configuration.q)
                rate.sleep()

    return data_x, data_y


def _generate_data(arg_list):
    return _do_generate_data(*arg_list)


def generate_data(n_data, nominal_lf_iso, nominal_rf_iso, nominal_sensor_data,
                  side, num_cpu):
    rollout_per_cpu = int(max(n_data // num_cpu, 1))
    args_list = [
        rollout_per_cpu, nominal_lf_iso, nominal_rf_iso, nominal_sensor_data,
        side
    ]
    results = util.try_multiprocess(args_list, num_cpu, _generate_data)

    data_x, data_y = [], []
    for result in results:
        data_x += result[0]
        data_y += result[1]

    return data_x, data_y


def save_weights_to_yaml(pytorch_model, path):
    mlp_model = dict()
    mlp_model['num_layer'] = len(pytorch_model.layers)
    for l_id, l in enumerate(pytorch_model.layers):
        if hasattr(l, 'weight'):
            mlp_model['w' + str(l_id)] = l.weight.tolist()
            mlp_model['b' + str(l_id)] = l.bias.reshape(
                1, l.weight.shape[0]).tolist()
        else:   # is activation function
            # Activation Fn Idx: None: 0, Tanh: 1
            if l_id == (len(pytorch_model.layers) - 1):
                mlp_model['act_fn' + str(l_id)] = 0
            else:
                mlp_model['act_fn' + str(l_id)] = 1

    with open(path + '/mlp_model.yaml', 'w') as f:
        yml = YAML()
        yml.dump(mlp_model, f)


def generate_casadi_func(pytorch_model,
                         input_mean,
                         input_std,
                         output_mean,
                         output_std,
                         generate_c_code=True):
    c_code_path = cwd + "/data/tf_model/atlas_crbi"
    ## Computational Graph
    b = MX.sym('b', 3)
    l = MX.sym('l', 3)
    r = MX.sym('r', 3)
    # Input
    l_minus_b = l - b
    r_minus_b = r - b
    inp = vertcat(l_minus_b, r_minus_b)
    normalized_inp = (inp - input_mean) / input_std  # (6, 1)
    # MLP (Somewhat manual)
    w0 = pytorch_model.layers[0].weight.detach().numpy()        # (6, 64)
    b0 = pytorch_model.layers[0].bias.detach().reshape(-1, 1).numpy()   # (1, 64)
    w1 = pytorch_model.layers[2].weight.detach().numpy()        # (64, 64)
    b1 = pytorch_model.layers[2].bias.detach().reshape(-1, 1).numpy()   # (1, 64)
    w2 = pytorch_model.layers[4].weight.detach().numpy()        # (64, 6)
    b2 = pytorch_model.layers[4].bias.detach().reshape(-1, 1).numpy()   # (6)

    output = mtimes(
        w2, tanh(mtimes(w1, tanh(mtimes(w0, normalized_inp) + b0)) + b1)
    ) + b2
    denormalized_output = (output.T @ output_std) + output_mean

    # Define casadi function
    func = Function('draco3_crbi_helper', [b, l, r], [denormalized_output])
    jac_func = func.jacobian()
    print(func)
    print(jac_func)

    if generate_c_code:
        # Code generator
        code_gen = CodeGenerator('draco3_crbi_helper.c', dict(with_header=True))
        code_gen.add(func)
        code_gen.add(jac_func)
        code_gen.generate()
        shutil.move(
            cwd + '/draco3_crbi_helper.h', cwd +
            "/experiment_data/draco3_crbi_helper.h"
        )
        shutil.move(
            cwd + '/draco3_crbi_helper.c',
            cwd + "/experiment_data/draco3_crbi_helper.c")

    return func, jac_func


def evaluate_crbi_model_using_casadi(cas_func, b, l, r):
    out = cas_func(b, l, r)
    return out


def evaluate_crbi_model_using_tf(tf_model, b, l, r, input_mean, input_std,
                                 output_mean, output_std):
    inp1 = l - b
    inp2 = r - b
    inp = np.concatenate([inp1, inp2], axis=0)
    normalized_inp = np.expand_dims(util.normalize(inp, input_mean, input_std),
                                    axis=0)
    output = tf_model(normalized_inp)
    d_output = util.denormalize(np.squeeze(output), output_mean, output_std)
    return d_output, output


if __name__ == "__main__":

    NUM_FLOATING_BASE = 5   # pinocchio model

    # initialize robot model
    robot = pin.RobotWrapper.BuildFromURDF(
        cwd + '/robot_model/draco/draco_modified.urdf',
        cwd + '/robot_model/draco',
        root_joint=pin.JointModelFreeFlyer())

    # Initialize meschcat visualizer
    if VISUALIZE:
        viz = pin.visualize.MeshcatVisualizer(robot.model, robot.collision_model,
                                              robot.visual_model)
        robot.setVisualizer(viz, init=False)
        viz.initViewer(open=True)
        viz.loadViewerModel()

    # Set initial robot configuration
    q0 = robot.q0.copy()
    q0[0:3] = np.array(INITIAL_POS_WORLD_TO_BASEJOINT)
    q0[3:7] = np.array(INITIAL_QUAT_WORLD_TO_BASEJOINT)
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
    # l_shoulder_aa_idx = robot.index("l_shldr_ie")
    # q0[NUM_FLOATING_BASE + l_shoulder_aa_idx] = np.pi / 2
    # r_shoulder_aa_idx = robot.index("r_shldr_ie")
    # q0[NUM_FLOATING_BASE + r_shoulder_aa_idx] = np.pi / 2
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
        cost=1e-1,  # [cost] / [rad]
    )

    # Joint coupling task
    r_knee_holonomic_task = JointCouplingTask(
        ["r_knee_fe_jp", "r_knee_fe_jd"],
        [1.0, -1.0],
        100.0,
        configuration,
        lm_damping=1e-7,
    )
    l_knee_holonomic_task = JointCouplingTask(
        ["l_knee_fe_jp", "l_knee_fe_jd"],
        [1.0, -1.0],
        100.0,
        configuration,
        lm_damping=1e-7,
    )

    tasks = [left_foot_task, torso_task, right_foot_task, posture_task,
             l_knee_holonomic_task, r_knee_holonomic_task]
    task_dict = {'torso_task': torso_task,
                 'lfoot_task': left_foot_task,
                 'rfoot_task': right_foot_task,
                 'posture_task': posture_task,
                 'lknee_constr_task': l_knee_holonomic_task,
                 'rknee_constr_task': r_knee_holonomic_task}

    nominal_base_iso = configuration.get_transform_frame_to_world(
        "torso_link").copy()
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

    CASE = 3

    # Set base_pos, base_quat, joint_pos here for visualization
    if CASE == 5:
        # Generate Dataset
        print("-" * 80)
        print("Pressed 5: Train CRBI Regressor w/multiprocessing")
        VISUALIZE = False   # can't do this with multiprocessing

        nominal_configuration = q0
        l_data_x, l_data_y = generate_data(N_MOTION_PER_LEG,
                                           nominal_lf_iso, nominal_rf_iso,
                                           nominal_configuration, "left",
                                           N_CPU_DATA_GEN)
        r_data_x, r_data_y = generate_data(N_MOTION_PER_LEG,
                                           nominal_lf_iso, nominal_rf_iso,
                                           nominal_configuration, "right",
                                           N_CPU_DATA_GEN)
        data_x = l_data_x + r_data_x
        data_y = l_data_y + r_data_y

        log_dir = "experiment_data/tensorboard/draco3_crbi_tf_copy"
        if os.path.exists(log_dir):
            shutil.rmtree(log_dir)
        writer = SummaryWriter(log_dir)

        ## normalize training data
        print("{} data is collected".format(len(data_x)))
        input_mean, input_std, input_normalized_data = util.normalize_data(
            data_x)
        output_mean, output_std, output_normalized_data = util.normalize_data(
            data_y)

        # create Torch data set
        x_data_torch, y_data_torch = torch.from_numpy(
            np.array(input_normalized_data)).float(), torch.from_numpy(
            np.array(output_normalized_data)).float()
        torch_dataset = torch_utils.TensorDataset(x_data_torch,
                                                  y_data_torch)

        # Split into train and test data sets
        train_size = int(len(torch_dataset) * 0.9)
        test_size = len(torch_dataset) - train_size
        train_dataset, test_dataset = torch.utils.data.random_split(torch_dataset,
                                                                    [train_size, test_size])

        # Load test / train data
        train_loader = torch_utils.DataLoader(dataset=train_dataset,
                                             batch_size=BATCH_SIZE,
                                             shuffle=True,
                                             num_workers=4)
        test_loader = torch_utils.DataLoader(dataset=test_dataset,
                                             batch_size=BATCH_SIZE,
                                             shuffle=False,
                                             num_workers=4)

        # Create NN model
        crbi_model = NetWork(6, 64, 64, 6)
        optimizer = torch.optim.SGD(crbi_model.parameters(), lr=LR, momentum=0.)
        loss_function = torch.nn.MSELoss()

        # train
        train_loss_per_epoch = 0.
        val_loss_per_epoch = 0.
        iter = 0
        for epoch in range(N_EPOCH):
            train_loss = 0.
            train_batch_loss = 0.
            crbi_model.train()
            for step, (b_x, b_y) in enumerate(train_loader):
                output = crbi_model(b_x)
                loss = loss_function(output, b_y)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                train_batch_loss = loss.item()
                train_loss += loss.item()
                iter += 1
                writer.add_scalar('batch loss', train_batch_loss, iter)
            train_loss_per_epoch = train_loss / len(train_loader)

        print('=' * 80)
        print('CRBI training done')

        # test
        crbi_model.eval()
        with torch.no_grad():
            test_loss_per_epoch = 0.
            for epoch in range(N_EPOCH):
                test_loss = 0.
                for i, (b_x, b_y) in enumerate(test_loader):
                    output = crbi_model(b_x)
                    loss = loss_function(output, b_y)
                    test_loss += loss.item()
                test_loss_per_epoch = test_loss / len(test_loader)
                writer.add_scalar('test loss vs epoch',
                                  test_loss_per_epoch, epoch + 1)

        print('=' * 80)
        print('CRBI test done')

        #####################################################
        '''plot centroidal inertia dim = 6'''
        #####################################################
        test_data_loader2 = torch_utils.DataLoader(
            dataset=test_dataset,
            batch_size=1,
            shuffle=False,
            num_workers=6)

        crbi_model.eval()
        gt_inertia_list, predict_inertia_list = [], []
        iter_list = []
        num_iter = 0
        with torch.no_grad():
            for i, (b_x, b_y) in enumerate(test_data_loader2):
                output = crbi_model(b_x)
                gt_denormalized_output = util.denormalize(
                    np.squeeze(b_y.numpy(), axis=0), output_mean,
                    output_std)
                predict_denormalized_output = util.denormalize(
                    np.squeeze(output.numpy(), axis=0), output_mean,
                    output_std)
                iter_list.append(num_iter)
                gt_inertia_list.append(gt_denormalized_output)
                predict_inertia_list.append(predict_denormalized_output)
                num_iter += 1

        ##plot
        fig, axes = plt.subplots(6, 1)
        for i in range(6):
            if i == 0:
                axes[i].plot(iter_list,
                             [gt_inertia[i] for gt_inertia in gt_inertia_list],
                             'r')
                axes[i].plot(iter_list, [
                    predict_inertia[i]
                    for predict_inertia in predict_inertia_list
                ], 'b')
                axes[i].grid(True)
            else:
                axes[i] = plt.subplot(6, 1, i + 1, sharex=axes[i-1])
                axes[i].plot(iter_list,
                             [gt_inertia[i] for gt_inertia in gt_inertia_list],
                             'r')
                axes[i].plot(iter_list, [
                    predict_inertia[i]
                    for predict_inertia in predict_inertia_list
                ], 'b')
                axes[i].grid(True)
        plt.show()

        # save model
        model_path = 'experiment_data/pytorch_model/draco3_crbi_tf_copy.pth'
        if os.path.exists(model_path):
            os.remove(model_path)
        torch.save(crbi_model, model_path)
        print("==============================================")
        print("Saved PyTorch Model State")
        print("==============================================")

        data_stats = {
            'input_mean': input_mean.tolist(),
            'input_std': input_std.tolist(),
            'output_mean': output_mean.tolist(),
            'output_std': output_std.tolist()
        }

        path = 'experiment_data/pytorch_model/draco3_crbi'
        with open('experiment_data/pytorch_model' + '/data_stat.yaml', 'w') as f:
            yml = YAML()
            yml.dump(data_stats, f)
        save_weights_to_yaml(crbi_model, path)
        print('=' * 80)
        print('Saved weights to yaml')

        cas_func, cas_jac_func = generate_casadi_func(
            crbi_model, input_mean, input_std, output_mean, output_std,
            True)

        exit(0)

    elif CASE == 4:
        print("-" * 80)
        print("Pressed 4: Load Pre-trained CRBI Model")
        crbi_model = torch.load('experiment_data/pytorch_model/draco3_crbi_tf_copy.pth')
        model_path = 'experiment_data/pytorch_model/draco3_crbi'
        with open(model_path + '/data_stat.yaml', 'r') as f:
            yml = YAML().load(f)
            input_mean = np.array(yml['input_mean'])
            input_std = np.array(yml['input_std'])
            output_mean = np.array(yml['output_mean'])
            output_std = np.array(yml['output_std'])

        b_regressor_trained = True

    elif CASE == 3:
        # Left Foot Swing, Right Foot Stance
        print("-" * 80)
        print("Pressed 3: Sample Motions - long footsteps (left side)")

        if VIDEO_RECORD:
            anim = meshcat.animation.Animation(default_framerate=1/DT)  # TODO fix rate
        _do_generate_data(N_DATA_PER_MOTION, nominal_lf_iso, nominal_rf_iso, q0, "left", 0.5)

        if VIDEO_RECORD:
            viz.viewer.set_animation(anim, play=False)

    elif CASE == 2:
        # Left Foot Stance, Right Foot Swing
        print("-" * 80)
        print("Pressed 2: Sample Motion for Right Foot Swing")

        swing_time, lfoot_ini_iso, lfoot_mid_iso, lfoot_fin_iso, lfoot_mid_vel, rfoot_ini_iso, rfoot_mid_iso, rfoot_fin_iso, rfoot_mid_vel, base_ini_iso, base_fin_iso = sample_swing_config(
            nominal_lf_iso, nominal_rf_iso, "right")
        s = 0.
        b_ik = True

