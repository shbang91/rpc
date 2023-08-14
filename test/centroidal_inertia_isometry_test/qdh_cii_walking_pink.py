#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""qdh humanoid standing on two feet and reaching with a hand."""

import numpy as np
import pinocchio as pin
import qpsolvers
from loop_rate_limiters import RateLimiter

import meshcat_shapes
import pink
from pink import solve_ik
from pink.tasks import FrameTask, JointCouplingTask, PostureTask

import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

from util.python_utils import interpolation
from util.python_utils import util
from util.python_utils import liegroup
from plot.data_saver import DataSaver

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description
except ModuleNotFoundError:
    raise ModuleNotFoundError("Examples need robot_descriptions, "
                              "try `pip install robot_descriptions`")

## parameters
INITIAL_POS = [0., 0., 0.660]
INITIAL_QUAT = [0., 0., 0., 1.]
NUM_FLOATING_BASE = 5
NORMALIZE_FACTOR = 1.

if __name__ == "__main__":
    # robot = load_robot_description("icub_description",
    # root_joint=pin.JointModelFreeFlyer())
    robot = pin.RobotWrapper.BuildFromURDF(
        cwd + '/robot_model/qdh/qdh_v01_simple.urdf',
        cwd + '/robot_model/qdh',
        root_joint=pin.JointModelFreeFlyer())

    # Initialize meschcat visualizer
    viz = pin.visualize.MeshcatVisualizer(robot.model, robot.collision_model,
                                          robot.visual_model)
    robot.setVisualizer(viz, init=False)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    # Set initial robot configuration
    q0 = robot.q0.copy()
    q0[0:3] = np.array(INITIAL_POS)
    q0[3:7] = np.array(INITIAL_QUAT)
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
    l_shoulder_aa_idx = robot.index("l_shldr_ie")
    # q0[NUM_FLOATING_BASE + l_shoulder_aa_idx] = np.pi / 2
    r_shoulder_aa_idx = robot.index("r_shldr_ie")
    # q0[NUM_FLOATING_BASE + r_shoulder_aa_idx] = np.pi / 2
    ## avoid joint violation
    configuration = pink.Configuration(robot.model, robot.data, q0)
    v0 = np.zeros(len(q0) - 1)
    viz.display(configuration.q)

    # calculate inertia at nominal pose
    _, _, nominal_inertia = robot.centroidal(q0, v0)
    nominal_rot_inertia = np.copy(nominal_inertia)[3:6, 3:6]

    # data saver for RCCRBI
    data_saver = DataSaver('qdh_cii_walking_pink.pkl')

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

    # tasks = [
    # left_foot_task, pelvis_task, torso_task, right_foot_task, posture_task
    # ]
    tasks = [left_foot_task, torso_task, right_foot_task, posture_task]

    nominal_base_iso = configuration.get_transform_frame_to_world(
        "Torso").copy()
    nominal_rf_iso = configuration.get_transform_frame_to_world("rFoot").copy()
    nominal_lf_iso = configuration.get_transform_frame_to_world("lFoot").copy()

    swing_time = 0.5
    ONE_STEP_X_LB, ONE_STEP_X_UB = 0.1 * NORMALIZE_FACTOR, 0.2 * NORMALIZE_FACTOR
    ONE_STEP_Y_LB, ONE_STEP_Y_UB = -0.1 * NORMALIZE_FACTOR, 0.1 * NORMALIZE_FACTOR
    SWING_HEIGHT = 0.10 * NORMALIZE_FACTOR
    NUM_NODE_PER_SWING = 30

    rate = RateLimiter(frequency=NUM_NODE_PER_SWING / swing_time)
    dt = rate.period
    print("dt: ", dt)

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"

    rfoot_contact_frame = viz.viewer["rFoot"]
    meshcat_shapes.frame(rfoot_contact_frame)
    # T_nominal_rf_iso = liegroup.RpToTrans(nominal_rf_iso.rotation,
    # nominal_rf_iso.translation)
    # rfoot_contact_frame.set_transform(T_nominal_rf_iso)
    for one_step_x in np.linspace(ONE_STEP_X_LB, ONE_STEP_X_UB, num=10):
        for one_step_y in np.linspace(ONE_STEP_Y_LB, ONE_STEP_Y_UB, num=10):

            final_rf_pos = nominal_rf_iso.translation + np.array(
                [one_step_x, one_step_y, 0])
            final_rf_ea = np.array([0., 0., 0.])
            final_rf_rot = util.euler_to_rot(final_rf_ea)
            final_rf_iso = liegroup.RpToTrans(final_rf_rot, final_rf_pos)

            nominal_rf_iso_liegroup = liegroup.RpToTrans(
                nominal_rf_iso.rotation, nominal_rf_iso.translation)
            mid_rf_iso = interpolation.iso_interpolate(nominal_rf_iso_liegroup,
                                                       final_rf_iso, 0.5)
            mid_rf_iso[2, 3] += SWING_HEIGHT
            mid_rf_vel = (final_rf_iso[:3, 3] -
                          nominal_rf_iso.translation) / swing_time

            mid_rf_vel[2] = 0.

            nominal_lf_iso_liegroup = liegroup.RpToTrans(
                nominal_lf_iso.rotation, nominal_lf_iso.translation)
            final_base_iso = interpolation.iso_interpolate(
                nominal_lf_iso_liegroup, final_rf_iso, 0.5)
            final_base_iso[2, 3] = nominal_base_iso.translation[2]

            #### Create curves using one step boundary
            base_pos_curve = interpolation.HermiteCurveVec(
                nominal_base_iso.translation, np.zeros(3), final_base_iso[:3,
                                                                          3],
                np.zeros(3), swing_time)
            base_quat_curve = interpolation.HermiteCurveQuat(
                util.rot_to_quat(nominal_base_iso.rotation), np.zeros(3),
                util.rot_to_quat(final_base_iso[:3, :3]), np.zeros(3),
                swing_time)

            rf_pos_curve_first_half = interpolation.HermiteCurveVec(
                nominal_rf_iso.translation, np.zeros(3), mid_rf_iso[:3, 3],
                mid_rf_vel, swing_time / 2.)
            rf_pos_curve_second_half = interpolation.HermiteCurveVec(
                mid_rf_iso[:3, 3], mid_rf_vel, final_rf_iso[:3, 3],
                np.zeros(3), swing_time / 2.)

            rf_quat_curve = interpolation.HermiteCurveQuat(
                util.rot_to_quat(nominal_rf_iso.rotation), np.zeros(3),
                util.rot_to_quat(final_rf_iso[:3, :3]), np.zeros(3),
                swing_time)

            #### prepare IK
            t = 0.0  # [s]
            for t in np.linspace(0., swing_time, num=NUM_NODE_PER_SWING):
                base_pos = base_pos_curve.evaluate(t)
                # base_quat = base_quat_curve.evaluate(t)
                base_quat = np.array(INITIAL_QUAT)

                if t <= swing_time / 2.:
                    rf_pos = rf_pos_curve_first_half.evaluate(t)
                else:
                    rf_pos = rf_pos_curve_second_half.evaluate(t -
                                                               swing_time / 2.)

                rf_quat = rf_quat_curve.evaluate(t)

                lf_pos = nominal_lf_iso.translation
                lf_quat = util.rot_to_quat(nominal_lf_iso.rotation)

                #### base
                T_w_base = liegroup.RpToTrans(util.quat_to_rot(base_quat),
                                              base_pos)
                #### right foot
                T_w_rf = liegroup.RpToTrans(util.quat_to_rot(rf_quat), rf_pos)

                #### left foot
                T_w_lf = liegroup.RpToTrans(util.quat_to_rot(lf_quat), lf_pos)

                # Task target specifications
                base_target_iso = pin.SE3(T_w_base[:3, :3], T_w_base[:3, 3])
                torso_task.set_target(base_target_iso)

                rf_target_iso = pin.SE3(T_w_rf[:3, :3], T_w_rf[:3, 3])
                right_foot_task.set_target(rf_target_iso)

                lf_target_iso = pin.SE3(T_w_lf[:3, :3], T_w_lf[:3, 3])
                left_foot_task.set_target(lf_target_iso)

                posture_task.set_target_from_configuration(configuration)

                #update visualizer
                rfoot_contact_frame.set_transform(T_w_rf)

                #### calculate IK
                # Compute velocity and integrate it into next configuration
                velocity = solve_ik(configuration, tasks, dt, solver=solver)
                configuration.integrate_inplace(velocity, dt)

                # compute CII
                _, _, centroidal_inertia = robot.centroidal(
                    configuration.q, v0)
                rot_inertia = np.copy(centroidal_inertia)[3:6, 3:6]
                CII = np.linalg.det(
                    np.dot(np.linalg.inv(rot_inertia), nominal_rot_inertia) -
                    np.eye(3))

                # save data
                data_saver.add("cii", CII)
                data_saver.advance()

                # Visualize result at fixed FPS
                viz.display(configuration.q)
                rate.sleep()
                t += dt

            # reset config
            configuration.q = q0
            configuration.update()
            viz.display(configuration.q)
            rfoot_contact_frame.set_transform(T_w_rf)
