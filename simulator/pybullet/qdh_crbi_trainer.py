#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import meshcat_shapes
import pink
from pink import solve_ik
from pink.tasks import FrameTask, JointCouplingTask, PostureTask

import numpy as np
import pinocchio as pin
import qpsolvers
from loop_rate_limiters import RateLimiter

import time
import copy
import math
from tqdm import tqdm
from ruamel.yaml import YAML
from casadi import *
import shutil

import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

from util.python_utils import interpolation
from util.python_utils import util
from util.python_utils import liegroup

from plot.data_saver import DataSaver

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import torch
import torch.utils.data as torch_utils
import torch.nn.functional as F
from torch.utils.tensorboard import SummaryWriter

## Configs
VISUALIZE = True
INITIAL_POS_WORLD_TO_BASEJOINT = [0, 0, 0.660]
INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0., 1.]

#motion boundary
SWING_TIME_LB, SWING_TIME_UB = 0.5, 1.
SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.03, 0.4

LFOOT_POS_DEV_LB = np.array([-0.15, -0.1, -0.05])
LFOOT_POS_DEV_UB = np.array([0.15, 0.1, 0.05])
RFOOT_POS_DEV_LB = np.array([-0.15, -0.1, -0.05])
RFOOT_POS_DEV_UB = np.array([0.15, 0.1, 0.05])

FOOT_EA_LB = np.array([np.deg2rad(-5.), np.deg2rad(-15.), np.deg2rad(-45)])
FOOT_EA_UB = np.array([np.deg2rad(5.), np.deg2rad(15.), np.deg2rad(45)])

BASE_HEIGHT_LB, BASE_HEIGHT_UB = 0.6, 0.7

## Data generation parameters
N_SWING_MOTIONS = 1000
N_DATA_PER_SWING = 15
N_CPU_USE_FOR_PARALELL_COM = 5

## learning hyperparameters
LR = 0.01
BATCH_SIZE = 32
EPOCH = 20


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


def sample_one_step_boundary(nominal_lf_iso, nominal_rf_iso, swing_foot_side):
    swing_time = np.random.uniform(SWING_TIME_LB, SWING_TIME_UB)
    swing_height = np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)
    min_y_distance_btw_foot = abs(nominal_lf_iso.translation[1] - nominal_rf_iso.translation[1])

    if swing_foot_side == "left_foot":
        #sample right foot
        initial_rf_iso = np.copy(nominal_rf_iso)
        final_rf_iso = np.copy(nominal_rf_iso)
        mid_rf_iso = interpolation.iso_interpolate(initial_rf_iso,
                                                   final_rf_iso, 0.5)
        mid_rf_vel = (final_rf_iso[0:3, 3] -
                      initial_rf_iso[0:3, 3]) / swing_time

        #sample left foot
        initial_lf_pos = nominal_lf_iso.translation + np.random.uniform(
            LFOOT_POS_DEV_LB, LFOOT_POS_DEV_UB)
        final_lf_pos = nominal_lf_iso.translation + np.random.uniform(
            LFOOT_POS_DEV_LB, LFOOT_POS_DEV_UB)

        while (abs(initial_lf_pos[1] - nominal_rf_iso.translation[1]) <
               min_y_distance_btw_foot):
            initial_lf_pos = nominal_lf_iso.translation + np.random.uniform(
                LFOOT_POS_DEV_LB, LFOOT_POS_DEV_UB)
        while (abs(final_lf_pos[1] - nominal_rf_iso.translation[1]) <
               min_y_distance_btw_foot):
            final_lf_pos = nominal_lf_iso.translation + np.random.uniform(
                LFOOT_POS_DEV_LB, LFOOT_POS_DEV_UB)

        initial_lf_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        initial_lf_rot = util.euler_to_rot(initial_lf_ea)
        final_lf_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        final_lf_rot = util.euler_to_rot(final_lf_ea)

        initial_lf_iso = liegroup.RpToTrans(initial_lf_rot, initial_lf_pos)
        final_lf_iso = liegroup.RpToTrans(final_lf_rot, final_lf_pos)
        mid_lf_iso = interpolation.iso_interpolate(initial_lf_iso,
                                                   final_lf_iso, 0.5)
        mid_lf_iso[2, 3] += swing_height
        mid_lf_vel = (final_lf_iso[0:3, 3] -
                      initial_lf_iso[0:3, 3]) / swing_time
        mid_lf_vel[2] = 0.

        #sample base
        initial_base_iso = interpolation.iso_interpolate(
            initial_lf_iso, initial_rf_iso, 0.5)
        initial_base_iso[2, 3] = np.random.uniform(BASE_HEIGHT_LB,
                                                   BASE_HEIGHT_UB)

        final_base_iso = interpolation.iso_interpolate(final_lf_iso,
                                                       final_rf_iso, 0.5)
        final_base_iso[2, 3] = np.random.uniform(BASE_HEIGHT_LB,
                                                 BASE_HEIGHT_UB)

    elif swing_foot_side == "right_foot":
        #sample left foot
        initial_lf_iso = np.copy(nominal_lf_iso)
        final_lf_iso = np.copy(nominal_lf_iso)
        mid_lf_iso = interpolation.iso_interpolate(initial_lf_iso,
                                                   final_lf_iso, 0.5)
        mid_lf_vel = (final_lf_iso[0:3, 3] -
                      initial_lf_iso[0:3, 3]) / swing_time

        #sample right foot
        initial_rf_pos = nominal_rf_iso.translation + np.random.uniform(
            RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)
        final_rf_pos = nominal_rf_iso.translation + np.random.uniform(
            RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)

        while (abs(initial_rf_pos[1] - nominal_lf_iso.translation[1]) <
               min_y_distance_btw_foot):
            initial_rf_pos = nominal_rf_iso.translation + np.random.uniform(
                RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)
        while (abs(final_rf_pos[1] - nominal_lf_iso.translation[1]) <
               min_y_distance_btw_foot):
            final_rf_pos = nominal_rf_iso.translation + np.random.uniform(
                RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)

        initial_rf_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        initial_rf_rot = util.euler_to_rot(initial_rf_ea)
        final_rf_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        final_rf_rot = util.euler_to_rot(final_rf_ea)

        initial_rf_iso = liegroup.RpToTrans(initial_rf_rot, initial_rf_pos)
        final_rf_iso = liegroup.RpToTrans(final_rf_rot, final_rf_pos)
        mid_rf_iso = interpolation.iso_interpolate(initial_rf_iso,
                                                   final_rf_iso, 0.5)
        mid_rf_iso[2, 3] += swing_height
        mid_rf_vel = (final_rf_iso[0:3, 3] -
                      initial_rf_iso[0:3, 3]) / swing_time
        mid_rf_vel[2] = 0.

        #sample base
        initial_base_iso = interpolation.iso_interpolate(
            initial_lf_iso, initial_rf_iso, 0.5)
        initial_base_iso[2, 3] = np.random.uniform(BASE_HEIGHT_LB,
                                                   BASE_HEIGHT_UB)

        final_base_iso = interpolation.iso_interpolate(final_lf_iso,
                                                       final_rf_iso, 0.5)
        final_base_iso[2, 3] = np.random.uniform(BASE_HEIGHT_LB,
                                                 BASE_HEIGHT_UB)

    else:
        raise ValueError

    return initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time


def create_curves_from_boundary(initial_base_iso, final_base_iso,
                                initial_lf_iso, mid_lf_iso, final_lf_iso,
                                mid_lf_vel, initial_rf_iso, mid_rf_iso,
                                final_rf_iso, mid_rf_vel, swing_time):

    base_pos_curve = interpolation.HermiteCurveVec(initial_base_iso[0:3, 3],
                                                   np.zeros(3),
                                                   final_base_iso[0:3, 3],
                                                   np.zeros(3), swing_time)

    base_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(initial_base_iso[0:3, 0:3]), np.zeros(3),
        util.rot_to_quat(final_base_iso[0:3, 0:3]), np.zeros(3), swing_time)

    lf_pos_curve_first_half = interpolation.HermiteCurveVec(
        initial_lf_iso[0:3, 3], np.zeros(3), mid_lf_iso[0:3, 3], mid_lf_vel,
        swing_time / 2.)

    lf_pos_curve_second_half = interpolation.HermiteCurveVec(
        mid_lf_iso[0:3, 3], mid_lf_vel, final_lf_iso[0:3, 3], np.zeros(3),
        swing_time / 2.)

    lf_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(initial_lf_iso[0:3, 0:3]), np.zeros(3),
        util.rot_to_quat(final_lf_iso[0:3, 0:3]), np.zeros(3), swing_time)

    rf_pos_curve_first_half = interpolation.HermiteCurveVec(
        initial_rf_iso[0:3, 3], np.zeros(3), mid_rf_iso[0:3, 3], mid_rf_vel,
        swing_time / 2.)

    rf_pos_curve_second_half = interpolation.HermiteCurveVec(
        mid_rf_iso[0:3, 3], mid_rf_vel, final_rf_iso[0:3, 3], np.zeros(3),
        swing_time / 2.)

    rf_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(initial_rf_iso[0:3, 0:3]), np.zeros(3),
        util.rot_to_quat(final_rf_iso[0:3, 0:3]), np.zeros(3), swing_time)

    return base_pos_curve, base_quat_curve, lf_pos_curve_first_half, lf_pos_curve_second_half, lf_quat_curve, rf_pos_curve_first_half, rf_pos_curve_second_half, rf_quat_curve

def generate_data_set(num_swing,
                      num_samples_per_swing,
                      nominal_lf_iso,
                      nominal_rf_iso,
                      nominal_configuration,
                      leg_side,
                      rseed=None,
                      cpu_idx=0):
    if rseed is not None:
        np.random.seed(rseed)
    x_data, y_data = [], []

    text = "{}".format(leg_side) + "#" + "{}".format(cpu_idx).zfill(2)
    with tqdm(total=num_swing * num_samples_per_swing,
              desc=text + 'data generation',
              position=cpu_idx) as pbar:
        for i in range(num_swing):
            initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time = sample_one_step_boundary(
                nominal_lf_iso, nominal_rf_iso, leg_side)

            base_pos_curve, base_quat_curve, lf_pos_curve_first_half, lf_pos_curve_second_half, lf_quat_curve, rf_pos_curve_first_half, rf_pos_curve_second_half, rf_quat_curve = create_curves_from_boundary(
                initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso,
                final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso,
                final_rf_iso, mid_rf_vel, swing_time)

            t = 0.0
            rate = RateLimiter(frequency=N_DATA_PER_SWING / swing_time)
            dt = rate.period
            for t in np.linspace(0, swing_time, num=num_samples_per_swing):
                base_pos = base_pos_curve.evaluate(t)
                base_quat = base_quat_curve.evaluate(t)

                if t <= swing_time / 2.:
                    lf_pos = lf_pos_curve_first_half.evaluate(t)
                    rf_pos = rf_pos_curve_first_half.evaluate(t)
                else:
                    lf_pos = lf_pos_curve_second_half.evaluate(t -
                                                               swing_time / 2.)
                    rf_pos = rf_pos_curve_second_half.evaluate(t -
                                                               swing_time / 2.)

                lf_quat = lf_quat_curve.evaluate(t)
                rf_quat = rf_quat_curve.evaluate(t)

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


                # solve ik
                # Compute velocity and integrate it into next configuration
                velocity = solve_ik(configuration, tasks, dt, solver=solver)
                configuration.integrate_inplace(velocity, dt)

                 # Visualize result at fixed FPS
                if VISUALIZE:
                    viz.display(configuration.q)
                    rate.sleep()
                t += dt

                # compute CRBI
                _, _, centroidal_inertia = robot.centroidal(configuration.q, v0)
                rot_inertia_in_world = np.copy(centroidal_inertia)[3:6, 3:6]

                rot_w_base = util.quat_to_rot(base_quat)
                rot_inertia_in_body = np.dot(np.dot(rot_w_base.transpose(), rot_inertia_in_world), rot_w_base)

                # append data (end-effector SE3 & body inertia pair)
                x_data.append(
                        np.concatenate([
                            lf_pos - base_pos, rf_pos - base_pos,
                        ],
                                       axis=0))
                y_data.append(
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


    return x_data, y_data


def do_generate_data_set(arg_list):
    return generate_data_set(*arg_list)


def parallerize_data_generate(num_swing, num_samples_per_swing, nominal_lf_iso,
                              nominal_rf_iso, nominal_configuration,
                              leg_side, num_cpu):
    data_x, data_y = [], []
    rollout_per_cpu = max((num_swing * num_samples_per_swing // num_cpu), 1)
    num_swing_per_cpu = rollout_per_cpu // num_samples_per_swing
    args_list = [
        num_swing_per_cpu, num_samples_per_swing, nominal_lf_iso,
        nominal_rf_iso, nominal_configuration, leg_side
    ]
    results = util.try_multiprocess(args_list, num_cpu, do_generate_data_set)
    for result in results:
        data_x += result[0]
        data_y += result[1]
    return data_x, data_y

def save_weights_to_yaml(pytorch_model):
    model_path = cwd + "/experiment_data/pytorch_model/qdh_crbi"
    mlp_model = dict()
    mlp_model['num_layer'] = len(pytorch_model.layers)
    for l_id, l in enumerate(pytorch_model.layers):
        mlp_model['w' + str(l_id)] = l.weights[0].numpy().tolist()
        mlp_model['b' + str(l_id)] = l.weights[1].numpy().reshape(
            1, l.weights[1].shape[0]).tolist()
        # Activation Fn Idx: None: 0, Tanh: 1
        if (l_id == (len(pytorch_model.layers) - 1)):
            mlp_model['act_fn' + str(l_id)] = 0
        else:
            mlp_model['act_fn' + str(l_id)] = 1
    with open(model_path + '/mlp_model.yaml', 'w') as f:
        yml = YAML()
        yml.dump(mlp_model, f)


def generate_casadi_func(pytorch_model,
                         input_mean,
                         input_std,
                         output_mean,
                         output_std,
                         generate_c_code=True):
    c_code_path = cwd + "/experiment_data/pytorch_model/qdh_crbi"
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

    w0 = pytorch_model.layers[0].weight.detach().numpy()  # (6, 64)
    b0 = pytorch_model.layers[0].bias.detach().numpy().reshape(-1, 1)  # (1, 64)
    w1 = pytorch_model.layers[2].weight.detach().numpy()  # (64, 64)
    b1 = pytorch_model.layers[2].bias.detach().numpy().reshape(-1, 1)  # (1, 64)
    w2 = pytorch_model.layers[4].weight.detach().numpy()  # (64, 6)
    b2 = pytorch_model.layers[4].bias.detach().numpy().reshape(-1, 1)  # (6)
    print("-----------------------------------------------")
    print(pytorch_model)
    print(w0.shape)
    print(b0.shape)
    print(w1.shape)
    print(b1.shape)
    print(w2.shape)
    print(b2.shape)
    # output = mtimes(
        # tanh(mtimes(tanh(mtimes(normalized_inp.T, w0) + b0), w1) + b1),
        # w2) + b2
    output = mtimes(
        w2, tanh(mtimes(w1, tanh(mtimes(w0, normalized_inp) + b0)) + b1)
        ) + b2
    denormalized_output = (output * output_std) + output_mean

    # Define casadi function
    func = Function('qdh_crbi_helper', [b, l, r], [denormalized_output])
    jac_func = func.jacobian()
    print(func)
    print(jac_func)

    if generate_c_code:
        # Code generator
        code_gen = CodeGenerator('qdh_crbi_helper.c', dict(with_header=True))
        code_gen.add(func)
        code_gen.add(jac_func)
        code_gen.generate()
        shutil.move(
            cwd + '/qdh_crbi_helper.h', cwd +
            "/experiment_data/qdh_crbi_helper.h"
        )
        shutil.move(
            cwd + '/qdh_crbi_helper.c',
            cwd + "/experiment_data/qdh_crbi_helper.c")

    return func, jac_func



#main loop
if __name__ == "__main__":

    NUM_FLOATING_BASE = 5 #pinocchio model

    # initialize robot model
    robot = pin.RobotWrapper.BuildFromURDF(
        cwd + '/robot_model/qdh/qdh_v01.urdf',
        cwd + '/robot_model/qdh',
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
    lknee_idx = robot.index("l_knee_fe")
    q0[NUM_FLOATING_BASE + lknee_idx] = np.pi / 2
    rknee_idx = robot.index("r_knee_fe")
    q0[NUM_FLOATING_BASE + rknee_idx] = np.pi / 2
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
        "lFoot",
        position_cost=10.0,
        orientation_cost=1.0,
    )
    torso_task = FrameTask(
        "Torso",
        position_cost=1.0,
        orientation_cost=1.0,
    )
    right_foot_task = FrameTask(
        "rFoot",
        position_cost=10.0,
        orientation_cost=1.0,
    )
    posture_task = PostureTask(
        cost=1e-1,  # [cost] / [rad]
    )

    tasks = [left_foot_task, torso_task, right_foot_task, posture_task]
    task_dict = {'torso_task': torso_task,
                 'lfoot_task': left_foot_task,
                 'rfoot_task': right_foot_task,
                 'posture_task': posture_task}

    nominal_base_iso = configuration.get_transform_frame_to_world(
        "Torso").copy()
    nominal_rf_iso = configuration.get_transform_frame_to_world("rFoot").copy()
    nominal_lf_iso = configuration.get_transform_frame_to_world("lFoot").copy()

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"

    # meshcat visualizer
    if VISUALIZE:
        rfoot_contact_frame = viz.viewer["rFoot"]
        meshcat_shapes.frame(rfoot_contact_frame)

        lfoot_contact_frame = viz.viewer["lFoot"]
        meshcat_shapes.frame(lfoot_contact_frame)

    ##TODO: make it as argument
    # case 0: right foot swing motion sampling
    # case 1: generate data set without multiprocessing and training
    # case 2: generate data set with multiprocessing and training CRBI model

    CASE = 2
    if (CASE == 0):
        print('=' * 80)
        print('right foot swing motion sampling')

        initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time = sample_one_step_boundary(
            nominal_lf_iso, nominal_rf_iso, 'right_foot')

        base_pos_curve, base_quat_curve, lf_pos_curve_first_half, lf_pos_curve_second_half, lf_quat_curve, rf_pos_curve_first_half, rf_pos_curve_second_half, rf_quat_curve = create_curves_from_boundary(
            initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso,
            final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso,
            final_rf_iso, mid_rf_vel, swing_time)


        t = 0.0
        rate = RateLimiter(frequency=N_DATA_PER_SWING / swing_time)
        dt = rate.period
        print("dt: ", dt)
        for t in np.linspace(0, swing_time, num=N_DATA_PER_SWING):

            base_pos = base_pos_curve.evaluate(t)
            base_quat = base_quat_curve.evaluate(t)

            if t <= swing_time / 2.:
                lf_pos = lf_pos_curve_first_half.evaluate(t)
                rf_pos = rf_pos_curve_first_half.evaluate(t)
            else:
                lf_pos = lf_pos_curve_second_half.evaluate(t -
                                                           swing_time / 2.)
                rf_pos = rf_pos_curve_second_half.evaluate(t -
                                                           swing_time / 2.)

            lf_quat = lf_quat_curve.evaluate(t)
            rf_quat = rf_quat_curve.evaluate(t)

            # set desired task
            des_base_iso = pin.SE3(util.quat_to_rot(base_quat), base_pos)
            task_dict['torso_task'].set_target(des_base_iso)

            des_rfoot_iso = pin.SE3(util.quat_to_rot(rf_quat), rf_pos)
            task_dict['rfoot_task'].set_target(des_rfoot_iso)

            des_lfoot_iso = pin.SE3(util.quat_to_rot(lf_quat), lf_pos)
            task_dict['lfoot_task'].set_target(des_lfoot_iso)

            #TODO: or set from configuration
            task_dict['posture_task'].set_target_from_configuration(configuration)

            # TODO: update meshcat visualizer
            #### base
            T_w_base = liegroup.RpToTrans(util.quat_to_rot(base_quat),
                                          base_pos)
            #### right foot
            T_w_rf = liegroup.RpToTrans(util.quat_to_rot(rf_quat), rf_pos)

            #### left foot
            T_w_lf = liegroup.RpToTrans(util.quat_to_rot(lf_quat), lf_pos)

            lfoot_contact_frame.set_transform(T_w_lf)
            rfoot_contact_frame.set_transform(T_w_rf)

            # solve ik
            # Compute velocity and integrate it into next configuration
            velocity = solve_ik(configuration, tasks, dt, solver=solver)
            configuration.integrate_inplace(velocity, dt)

            # Visualize result at fixed FPS
            viz.display(configuration.q)
            rate.sleep()
            t += dt

        # reset config
        configuration.q = q0
        configuration.update()
        viz.display(configuration.q)

    elif CASE == 1:
        print('=' * 80)
        print('generate data set without multiprocessing and training')
        nominal_configuration = q0
        x_data, y_data = generate_data_set(N_SWING_MOTIONS,
                                           N_DATA_PER_SWING,
                                           nominal_lf_iso, nominal_rf_iso,
                                           nominal_configuration,
                                           'right_foot')
        x_data_test, y_data_test = generate_data_set(
            (N_SWING_MOTIONS // 10), N_DATA_PER_SWING, nominal_lf_iso,
            nominal_rf_iso, nominal_configuration, 'right_foot')

        print('{} training data set collected'.format(len(x_data)))
        print('{} test data set collected'.format(len(x_data_test)))

        ## normalize training data
        input_mean, input_std, input_normalized_data = util.normalize_data(
            x_data)
        output_mean, output_std, output_normalized_data = util.normalize_data(
            y_data)

        x_data_torch, y_data_torch = torch.from_numpy(
            np.array(input_normalized_data)).float(), torch.from_numpy(
                np.array(output_normalized_data)).float()
        torch_dataset = torch_utils.TensorDataset(x_data_torch,
                                                  y_data_torch)
        data_loader = torch_utils.DataLoader(dataset=torch_dataset,
                                             batch_size=BATCH_SIZE,
                                             shuffle=True,
                                             num_workers=2)

        crbi_model = NetWork(6, 64, 64, 6)
        optimizer = torch.optim.SGD(crbi_model.parameters(), lr=LR)
        loss_function = torch.nn.MSELoss()

        log_dir = "experiment_data/tensorboard/qdh_crbi"
        if os.path.exists(log_dir):
            shutil.rmtree(log_dir)
        writer = SummaryWriter(log_dir)

        ## training
        # loss_history = []
        loss_idx_value = 0
        for epoch in range(EPOCH):
            for step, (b_x, b_y) in enumerate(data_loader):
                output = crbi_model(b_x)
                loss = loss_function(output, b_y)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                # loss_history.append(loss.data.numpy())
                current_loss = loss.item()
                writer.add_scalar("Loss", current_loss, loss_idx_value)
                loss_idx_value += 1

        print('training done')

        ## test model
        ## normalizing test set
        test_input_normalized_data = util.normalize(
            x_data_test, input_mean, input_std)
        test_output_normalized_data = util.normalize(
            y_data_test, output_mean, output_std)

        test_x_data_torch, test_y_data_torch = torch.from_numpy(
            np.array(
                test_input_normalized_data)).float(), torch.from_numpy(
                    np.array(test_output_normalized_data)).float()
        test_torch_dataset = torch_utils.TensorDataset(
            test_x_data_torch, test_y_data_torch)
        test_data_loader = torch_utils.DataLoader(
            dataset=test_torch_dataset,
            batch_size=BATCH_SIZE,
            shuffle=False)

        test_loss_idx = 0
        crbi_model.eval()
        with torch.no_grad():
            for i, (b_x, b_y) in enumerate(test_data_loader):
                output = crbi_model(b_x)
                loss = loss_function(output, b_y)
                test_loss = loss.item()
                writer.add_scalar("TEST Loss", test_loss, test_loss_idx)
                test_loss_idx += 1

        print('test done')

        exit(0)

        ## plot loss history
        # plt.plot(loss_history)
        # plt.show()
    elif CASE == 2:
        print('=' * 80)
        print(
            'Pressed 5: generate data set with multiprocessing and training CRBI model'
        )
        VISUALIZE = False

        ##################################################################
        '''DATA Generation'''
        ##################################################################
        nominal_configuration = q0
        x_data_lf, y_data_lf = parallerize_data_generate(
            N_SWING_MOTIONS, N_DATA_PER_SWING, nominal_lf_iso,
            nominal_rf_iso, nominal_configuration, 'left_foot',
            N_CPU_USE_FOR_PARALELL_COM)

        x_data_rf, y_data_rf = parallerize_data_generate(
            N_SWING_MOTIONS, N_DATA_PER_SWING, nominal_lf_iso,
            nominal_rf_iso, nominal_configuration, 'right_foot',
            N_CPU_USE_FOR_PARALELL_COM)

        x_data_lf_val, y_data_lf_val = parallerize_data_generate(
            (N_SWING_MOTIONS // 10), N_DATA_PER_SWING, nominal_lf_iso,
            nominal_rf_iso, nominal_configuration, 'left_foot',
            N_CPU_USE_FOR_PARALELL_COM)

        x_data_rf_val, y_data_rf_val = parallerize_data_generate(
            (N_SWING_MOTIONS // 10), N_DATA_PER_SWING, nominal_lf_iso,
            nominal_rf_iso, nominal_configuration, 'right_foot',
            N_CPU_USE_FOR_PARALELL_COM)

        x_data_lf_test, y_data_lf_test = parallerize_data_generate(
            (N_SWING_MOTIONS // 50), N_DATA_PER_SWING, nominal_lf_iso,
            nominal_rf_iso, nominal_configuration, 'left_foot',
            N_CPU_USE_FOR_PARALELL_COM)

        x_data_rf_test, y_data_rf_test = parallerize_data_generate(
            (N_SWING_MOTIONS // 50), N_DATA_PER_SWING, nominal_lf_iso,
            nominal_rf_iso, nominal_configuration, 'right_foot',
            N_CPU_USE_FOR_PARALELL_COM)

        x_data = x_data_lf + x_data_rf
        y_data = y_data_lf + y_data_rf

        x_data_val = x_data_lf_val + x_data_rf_val
        y_data_val = y_data_lf_val + y_data_lf_val

        x_data_test = x_data_lf_test + x_data_rf_test
        y_data_test = y_data_lf_test + y_data_lf_test

        ##################################################################
        '''Training Dataset Normalization'''
        ##################################################################

        mean_x_data, std_x_data, normalized_x_data = util.normalize_data(
            x_data)
        mean_y_data, std_y_data, normalized_y_data = util.normalize_data(
            y_data)

        ##################################################################
        '''Pytorch Dataloader'''
        ##################################################################

        x_data_torch, y_data_torch = torch.from_numpy(
            np.array(normalized_x_data)).float(), torch.from_numpy(
                np.array(normalized_y_data)).float()
        dataset_torch = torch_utils.TensorDataset(x_data_torch,
                                                  y_data_torch)
        data_loader = torch_utils.DataLoader(dataset=dataset_torch,
                                             batch_size=BATCH_SIZE,
                                             shuffle=True,
                                             num_workers=6)

        normalized_x_data_val = util.normalize(x_data_val, mean_x_data,
                                               std_x_data)
        normalized_y_data_val = util.normalize(y_data_val, mean_y_data,
                                               std_y_data)
        x_data_val_torch, y_data_val_torch = torch.from_numpy(
            np.array(normalized_x_data_val)).float(), torch.from_numpy(
                np.array(normalized_y_data_val)).float()

        val_torch_dataset = torch_utils.TensorDataset(
            x_data_val_torch, y_data_val_torch)
        val_data_loader = torch_utils.DataLoader(dataset=val_torch_dataset,
                                                 batch_size=BATCH_SIZE,
                                                 shuffle=False,
                                                 num_workers=6)

        ##################################################################
        '''Training Regressor'''
        ##################################################################
        #regressor model
        nn_crbi_model = NetWork(6, 64, 64, 6)

        #loss & optimizer
        loss_function = torch.nn.MSELoss()
        optimizer = torch.optim.SGD(nn_crbi_model.parameters(), lr=LR)

        # add tensorboard for visualization
        log_dir = "experiment_data/tensorboard/qdh_crbi"
        if os.path.exists(log_dir):
            shutil.rmtree(log_dir)
        writer = SummaryWriter(log_dir)

        # training
        train_loss_per_epoch = 0.
        val_loss_per_epoch = 0.
        iter = 0
        for epoch in range(EPOCH):
            train_loss = 0.
            train_batch_loss = 0.
            nn_crbi_model.train()
            for step, (b_x, b_y) in enumerate(data_loader):
                output = nn_crbi_model(b_x)
                loss = loss_function(output, b_y)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                train_batch_loss = loss.item()
                train_loss += loss.item()
                iter += 1
                writer.add_scalar('batch loss', train_batch_loss, iter)
            train_loss_per_epoch = train_loss / len(data_loader)

            val_loss = 0.
            nn_crbi_model.eval()
            for step, (b_x, b_y) in enumerate(val_data_loader):
                output = nn_crbi_model(b_x)
                loss = loss_function(output, b_y)
                val_loss += loss.item()
            val_loss_per_epoch = train_loss / len(val_data_loader)

            writer.add_scalars(
                'loss vs epoch', {
                    'train': train_loss_per_epoch,
                    'validation': val_loss_per_epoch
                }, epoch + 1)



        print('=' * 80)
        print("CRBI training done")

        ##################################################################
        '''Test Regressor'''
        ##################################################################
        normalized_x_data_test = util.normalize(x_data_test, mean_x_data,
                                                std_x_data)
        normalized_y_data_test = util.normalize(y_data_test, mean_y_data,
                                                std_y_data)
        x_data_test_torch, y_data_test_torch = torch.from_numpy(
            np.array(normalized_x_data_test)).float(), torch.from_numpy(
                np.array(normalized_y_data_test)).float()

        test_torch_dataset = torch_utils.TensorDataset(
            x_data_test_torch, y_data_test_torch)
        test_data_loader = torch_utils.DataLoader(
            dataset=test_torch_dataset,
            batch_size=BATCH_SIZE,
            shuffle=False,
            num_workers=6)

        nn_crbi_model.eval()
        with torch.no_grad():
            test_loss_per_epoch = 0.
            for epoch in range(EPOCH):
                test_loss = 0.
                for i, (b_x, b_y) in enumerate(test_data_loader):
                    output = nn_crbi_model(b_x)
                    loss = loss_function(output, b_y)
                    test_loss += loss.item()
                test_loss_per_epoch = test_loss / len(test_data_loader)

                writer.add_scalar('test loss vs epoch',
                                  test_loss_per_epoch, epoch + 1)

        print('=' * 80)
        print('CRBI test done')

        #####################################################
        '''plot centroidal inertia dim = 6'''
        #####################################################
        test_data_loader2 = torch_utils.DataLoader(
            dataset=test_torch_dataset,
            batch_size=1,
            shuffle=False,
            num_workers=6)

        nn_crbi_model.eval()
        gt_inertia_list, predict_inertia_list = [], []
        iter_list = []
        num_iter = 0
        with torch.no_grad():
            for i, (b_x, b_y) in enumerate(test_data_loader2):
                output = nn_crbi_model(b_x)
                gt_denormalized_output = util.denormalize(
                    np.squeeze(b_y.numpy(), axis=0), mean_y_data,
                    std_y_data)
                predict_denormalized_output = util.denormalize(
                    np.squeeze(output.numpy(), axis=0), mean_y_data,
                    std_y_data)
                iter_list.append(num_iter)
                gt_inertia_list.append(gt_denormalized_output)
                predict_inertia_list.append(predict_denormalized_output)
                num_iter += 1

        # save the model
        model_path = cwd + '/experiment_data/pytorch_model/qdh_crbi'
        if not os.path.exists(model_path):
            os.makedirs(model_path)
        torch.save(nn_crbi_model, model_path + '/qdh_crbi.pth')
        print("==============================================")
        print("Saved PyTorch Model State")
        print("==============================================")

        data_stats = {
                'input_mean' : mean_x_data.tolist(),
                'input_std' : std_x_data.tolist(),
                'output_mean' : mean_y_data.tolist(),
                'output_std' : std_y_data.tolist()
                }
        # with open(model_path + '/data_stat.yaml', 'w') as f:
            # yml = YAML()
            # yml.dump(data_stats, f)
        # save_weights_to_yaml(nn_crbi_model)

        cas_func, cas_jac_func = generate_casadi_func(
            nn_crbi_model, mean_x_data, std_x_data, mean_y_data, std_y_data,
            True)

        ##plot
        fig, axes = plt.subplots(6, 1)
        for i in range(6):
            axes[i].plot(iter_list,
                         [gt_inertia[i] for gt_inertia in gt_inertia_list],
                         'r')
            axes[i].plot(iter_list, [
                predict_inertia[i]
                for predict_inertia in predict_inertia_list
            ], 'b')
            axes[i].grid(True)
        plt.show()

        exit(0)
