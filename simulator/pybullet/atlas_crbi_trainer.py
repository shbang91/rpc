import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import pybullet as pb
import numpy as np

import time
import copy
import math
from tqdm import tqdm

from util.python_utils import pybullet_util
from util.python_utils import interpolation
from util.python_utils import util
from util.python_utils import liegroup
from util.python_utils import robot_kinematics

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

## Configs
VIDEO_RECORD = False
PRINT_FREQ = 10
DT = 0.01
PRINT_ROBOT_INFO = False
INITIAL_POS_WORLD_TO_BASEJOINT = [0, 0, 1.5 - 0.761]
INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0., 1.]

#motion boundary
SWING_TIME_LB, SWING_TIME_UB = 0.35, 0.7

SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.03, 0.3

LFOOT_POS_DEV_LB = np.array([-0.15, -0.1, -0.05])
LFOOT_POS_DEV_UB = np.array([0.15, 0.1, 0.05])
RFOOT_POS_DEV_LB = np.array([-0.15, -0.1, -0.05])
RFOOT_POS_DEV_UB = np.array([0.15, 0.1, 0.05])

FOOT_EA_LB = np.array([np.deg2rad(-5.), np.deg2rad(-15.), np.deg2rad(-45)])
FOOT_EA_UB = np.array([np.deg2rad(5.), np.deg2rad(15.), np.deg2rad(45)])

BASE_HEIGHT_LB, BASE_HEIGHT_UB = 0.7, 0.8

## Data generation parameters
N_SWING_MOTIONS = 100
N_DATA_PER_SWING = 100
N_CPU_USE_FOR_PARALELL_COM = 5


def set_initial_config(robot, joint_id):
    # shoulder_x
    pb.resetJointState(robot, joint_id["l_arm_shx"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_arm_shx"], np.pi / 4, 0.)
    # elbow_y
    pb.resetJointState(robot, joint_id["l_arm_ely"], -np.pi / 2, 0.)
    pb.resetJointState(robot, joint_id["r_arm_ely"], np.pi / 2, 0.)
    # elbow_x
    pb.resetJointState(robot, joint_id["l_arm_elx"], -np.pi / 2, 0.)
    pb.resetJointState(robot, joint_id["r_arm_elx"], -np.pi / 2, 0.)
    # hip_y
    pb.resetJointState(robot, joint_id["l_leg_hpy"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_leg_hpy"], -np.pi / 4, 0.)
    # knee
    pb.resetJointState(robot, joint_id["l_leg_kny"], np.pi / 2, 0.)
    pb.resetJointState(robot, joint_id["r_leg_kny"], np.pi / 2, 0.)
    # ankle
    pb.resetJointState(robot, joint_id["l_leg_aky"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_leg_aky"], -np.pi / 4, 0.)


def sample_one_step_boundary(nominal_lf_iso, nominal_rf_iso, swing_foot_side):
    swing_time = np.random.uniform(SWING_TIME_LB, SWING_TIME_UB)
    swing_height = np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)
    min_y_distance_btw_foot = abs(nominal_lf_iso[1, 3] - nominal_rf_iso[1, 3])

    if swing_foot_side == "left_foot":
        #sample right foot
        initial_rf_iso = np.copy(nominal_rf_iso)
        final_rf_iso = np.copy(nominal_rf_iso)
        mid_rf_iso = interpolation.iso_interpolate(initial_rf_iso,
                                                   final_rf_iso, 0.5)
        mid_rf_vel = (final_rf_iso[0:3, 3] -
                      initial_rf_iso[0:3, 3]) / swing_time

        #sample left foot
        initial_lf_pos = nominal_lf_iso[0:3, 3] + np.random.uniform(
            LFOOT_POS_DEV_LB, LFOOT_POS_DEV_UB)
        final_lf_pos = nominal_lf_iso[0:3, 3] + np.random.uniform(
            LFOOT_POS_DEV_LB, LFOOT_POS_DEV_UB)

        while (abs(initial_lf_pos[1] - nominal_rf_iso[1, 3]) <
               min_y_distance_btw_foot):
            initial_lf_pos = nominal_lf_iso[0:3, 3] + np.random.uniform(
                LFOOT_POS_DEV_LB, LFOOT_POS_DEV_UB)
        while (abs(final_lf_pos[1] - nominal_rf_iso[1, 3]) <
               min_y_distance_btw_foot):
            final_lf_pos = nominal_lf_iso[0:3, 3] + np.random.uniform(
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
        initial_rf_pos = nominal_rf_iso[0:3, 3] + np.random.uniform(
            RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)
        final_rf_pos = nominal_rf_iso[0:3, 3] + np.random.uniform(
            RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)

        while (abs(initial_rf_pos[1] - nominal_lf_iso[1, 3]) <
               min_y_distance_btw_foot):
            initial_rf_pos = nominal_rf_iso[0:3, 3] + np.random.uniform(
                RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)
        while (abs(final_rf_pos[1] - nominal_lf_iso[1, 3]) <
               min_y_distance_btw_foot):
            final_rf_pos = nominal_rf_iso[0:3, 3] + np.random.uniform(
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


def lowerbody_inverse_kinematics(base_pos, base_quat, lf_pos, lf_quat, rf_pos,
                                 rf_quat, open_chain_joints_name_dict,
                                 joint_screws_in_ee_at_home, ee_SE3_at_home,
                                 nominal_sensor_data_dict):
    """
    param: open_chain_joints_name_dict (dict): left_leg, right_leg, left_arm, right_arm
    param: joint_screws_in_ee_at_home (dict): left_leg, right_leg, left_arm, right_arm
    param: ee_SE3_at_home (dict): left_leg, right_leg, left_arm, right_arm
    """
    joint_pos = copy.deepcopy(nominal_sensor_data_dict['joint_pos'])
    T_w_base = liegroup.RpToTrans(util.quat_to_rot(base_quat), base_pos)
    ##left_leg ik
    T_w_lf = liegroup.RpToTrans(util.quat_to_rot(lf_quat), lf_pos)
    des_T_base_lf = np.dot(liegroup.TransInv(T_w_base), T_w_lf)
    lf_initial_guess = np.array([
        joint_pos[joint_name]
        for joint_name in open_chain_joints_name_dict['left_leg']
    ])
    lf_joint_pos_sol, lf_ik_success = robot_kinematics.IKinBody(
        joint_screws_in_ee_at_home['left_leg'], ee_SE3_at_home['left_leg'],
        des_T_base_lf, lf_initial_guess)
    for i, joint_name in enumerate(open_chain_joints_name_dict['left_leg']):
        joint_pos[joint_name] = lf_joint_pos_sol[i]

    ##right_leg ik
    T_w_rf = liegroup.RpToTrans(util.quat_to_rot(rf_quat), rf_pos)
    des_T_base_rf = np.dot(liegroup.TransInv(T_w_base), T_w_rf)
    rf_initial_guess = np.array([
        joint_pos[joint_name]
        for joint_name in open_chain_joints_name_dict['right_leg']
    ])
    rf_joint_pos_sol, rf_ik_success = robot_kinematics.IKinBody(
        joint_screws_in_ee_at_home['right_leg'], ee_SE3_at_home['right_leg'],
        des_T_base_rf, rf_initial_guess)
    for i, joint_name in enumerate(open_chain_joints_name_dict['right_leg']):
        joint_pos[joint_name] = rf_joint_pos_sol[i]

    return joint_pos, lf_ik_success, rf_ik_success


def get_centroidal_rot_inertia_in_body(robot_system, base_com_pos,
                                       base_com_quat, joint_pos):
    rot_w_base_com = util.quat_to_rot(np.copy(base_com_quat))
    rot_w_basejoint = np.dot(rot_w_base_com,
                             rot_basejoint_to_basecom.transpose())
    base_joint_pos = base_com_pos - np.dot(rot_w_basejoint,
                                           pos_basejoint_to_basecom)
    #update pinocchio robotsystem
    robot_system.update_system(base_com_pos, base_com_quat, np.zeros(3),
                               np.zeros(3), base_joint_pos,
                               util.rot_to_quat(rot_w_basejoint), np.zeros(3),
                               np.zeros(3), joint_pos,
                               nominal_sensor_data_dict['joint_vel'], True)
    I_w = np.copy(robot_system.Ig[0:3, 0:3])
    I_b = np.dot(np.dot(rot_w_base_com.transpose(), I_w), rot_w_base_com)
    return I_b


def generate_data_set(num_swing,
                      num_samples_per_swing,
                      lf_nominal_pos,
                      rf_nominal_pos,
                      nominal_sensor_data_dict,
                      leg_side,
                      rseed=None,
                      cpu_idx=0):
    if rseed is not None:
        np.random.seed(rseed)
    x_data, y_data = [], []

    text = "#" + "{}".format(cpu_idx).zfill(2)
    with tqdm(total=num_swing * num_samples_per_swing,
              desc=text + 'data generation',
              position=cpu_idx) as pbar:
        for i in range(num_swing):
            initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time = sample_one_step_boundary(
                lf_nominal_pos, rf_nominal_pos, leg_side)

            base_pos_curve, base_quat_curve, lf_pos_curve_first_half, lf_pos_curve_second_half, lf_quat_curve, rf_pos_curve_first_half, rf_pos_curve_second_half, rf_quat_curve = create_curves_from_boundary(
                initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso,
                final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso,
                final_rf_iso, mid_rf_vel, swing_time)

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

                joint_pos, lf_ik_success, rf_ik_success = lowerbody_inverse_kinematics(
                    base_pos, base_quat, lf_pos, lf_quat, rf_pos, rf_quat,
                    open_chain_joints_name_dict, joint_screws_in_ee_at_home,
                    ee_SE3_at_home, nominal_sensor_data_dict)

                if lf_ik_success and rf_ik_success:
                    from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem
                    robot_system = PinocchioRobotSystem(
                        cwd + "/robot_model/atlas/atlas.urdf",
                        cwd + "/robot_model/atlas", False, False)

                    I_b = get_centroidal_rot_inertia_in_body(
                        robot_system, base_pos, base_quat, joint_pos)

                    base_euler = (R.from_quat(base_quat)).as_euler(
                        'xyz', degrees=True)
                    lf_euler = (R.from_quat(lf_quat)).as_euler('xyz',
                                                               degrees=True)
                    rf_euler = (R.from_quat(rf_quat)).as_euler('xyz',
                                                               degrees=True)

                    x_data.append(
                        np.concatenate([
                            base_pos, base_euler, lf_pos, lf_euler, rf_pos,
                            rf_euler
                        ],
                                       axis=0))
                    y_data.append(
                        np.array([
                            I_b[0, 0], I_b[1, 1], I_b[2, 2], I_b[0, 1],
                            I_b[0, 2], I_b[1, 2]
                        ]))
                    ##visualize data generation
                    # pybullet_util.set_config(robot, joint_id, base_pos,
                    # base_quat, joint_pos)
                    pbar.update(1)

    return data_x, data_y


def do_generate_data_set(arg_list):
    return generate_data_set(*arg_list)


def parallerize_data_generate(num_swing, num_samples_per_swing, lf_nominal_pos,
                              rf_nominal_pos, nominal_sensor_data_dict,
                              leg_side, num_cpu):
    data_x, data_y = [], []
    rollout_per_cpu = max((num_swing * num_samples_per_swing // num_cpu), 1)
    num_swing_per_cpu = rollout_per_cpu // num_samples_per_swing
    args_list = [
        num_swing_per_cpu, num_samples_per_swing, lf_nominal_pos,
        rf_nominal_pos, nominal_sensor_data_dict, leg_side
    ]
    results = util.try_multiprocess(args_list, num_cpu, do_generate_data_set)
    for result in results:
        data_x += result[0]
        data_y += result[1]
    return data_x, data_y


#main loop
if __name__ == "__main__":

    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=1.5,
                                  cameraYaw=120,
                                  cameraPitch=-30,
                                  cameraTargetPosition=[1, 0.5, 1.5])
    ## sim physics setting
    # pb.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1)
    # pb.setGravity(0, 0, -9.81)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    robot = pb.loadURDF(cwd + "/robot_model/atlas/atlas.urdf",
                        INITIAL_POS_WORLD_TO_BASEJOINT,
                        INITIAL_QUAT_WORLD_TO_BASEJOINT,
                        useFixedBase=False)

    # ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf",
    # useFixedBase=True)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot, INITIAL_POS_WORLD_TO_BASEJOINT, INITIAL_QUAT_WORLD_TO_BASEJOINT,
        PRINT_ROBOT_INFO)

    ##prepare inverse kinematics
    joint_screws_in_ee_at_home, ee_SE3_at_home = dict(), dict()
    open_chain_joints_name_dict, ee_links_name_dict, base_links_name_dict = dict(
    ), dict(), dict()

    leg_list = ['left_leg', 'right_leg']
    # limbs_list = ["left_leg", "right_leg", "left_arm", "right_arm"]

    open_chain_joints_name_dict['left_leg'] = [
        'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky',
        'l_leg_akx'
    ]
    open_chain_joints_name_dict['right_leg'] = [
        'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky',
        'r_leg_akx'
    ]
    # open_chain_joints_name_dict['left_arm'] = []
    # open_chain_joints_name_dict['right_arm'] = []

    ee_links_name_dict['left_leg'] = 'l_sole'
    ee_links_name_dict['right_leg'] = 'r_sole'
    # ee_links_name_dict['left_arm'] =
    # ee_links_name_dict['right_arm'] =

    base_links_name_dict['left_leg'] = 'pelvis'
    base_links_name_dict['right_leg'] = 'pelvis'

    for leg in leg_list:
        joint_screws_in_ee_at_home[leg], ee_SE3_at_home[
            leg] = robot_kinematics.get_kinematics_config(
                robot, joint_id, link_id, open_chain_joints_name_dict[leg],
                base_links_name_dict[leg], ee_links_name_dict[leg])

    ##robot initial config setting
    set_initial_config(robot, joint_id)

    ##robot joint and link dynamics setting
    pybullet_util.set_joint_friction(robot, joint_id, 0)
    pybullet_util.set_link_damping(robot, link_id, 0., 0.)

    ##get sensor data
    nominal_sensor_data_dict = pybullet_util.get_sensor_data(
        robot, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)
    lf_nominal_pos = pybullet_util.get_link_iso(robot, link_id['l_sole'])
    rf_nominal_pos = pybullet_util.get_link_iso(robot, link_id['r_sole'])
    nominal_base_pos = np.copy(nominal_sensor_data_dict['base_com_pos'])
    nominal_base_quat = np.copy(nominal_sensor_data_dict['base_com_quat'])
    nominal_joint_pos = copy.deepcopy(nominal_sensor_data_dict['joint_pos'])

    ##sim
    t = 0.
    dt = DT
    count = 0

    loop_count = 0

    while (True):

        # Get Keyboard Event
        keys = pb.getKeyboardEvents()

        if pybullet_util.is_key_triggered(keys, '1'):
            ## reset to nominal pose
            pybullet_util.set_config(robot, joint_id, nominal_base_pos,
                                     nominal_base_quat, nominal_joint_pos)

        elif pybullet_util.is_key_triggered(keys, '2'):

            print('=' * 80)
            print('left foot swing motion sampling')

            initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time = sample_one_step_boundary(
                lf_nominal_pos, rf_nominal_pos, 'left_foot')

            base_pos_curve, base_quat_curve, lf_pos_curve_first_half, lf_pos_curve_second_half, lf_quat_curve, rf_pos_curve_first_half, rf_pos_curve_second_half, rf_quat_curve = create_curves_from_boundary(
                initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso,
                final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso,
                final_rf_iso, mid_rf_vel, swing_time)

            pbar = tqdm(total=math.floor(swing_time))

            # base_pos_list, rf_pos_list, rf_quat_list, lf_pos_list, time_list = [], [], [], [], []
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

                joint_pos, lf_ik_success, rf_ik_success = lowerbody_inverse_kinematics(
                    base_pos, base_quat, lf_pos, lf_quat, rf_pos, rf_quat,
                    open_chain_joints_name_dict, joint_screws_in_ee_at_home,
                    ee_SE3_at_home, nominal_sensor_data_dict)

                # prepare plotting curve
                # time_list.append(t)
                # lf_pos_list.append(lf_pos)
                # rf_pos_list.append(rf_pos)
                # rf_quat_list.append(rf_quat)
                # base_pos_list.append(base_pos)
                if not (lf_ik_success and rf_ik_success):
                    print('ik fail to solve')
                    __import__('ipdb').set_trace()

                # visualize config
                pybullet_util.set_config(robot, joint_id, base_pos, base_quat,
                                         joint_pos)
                if t < swing_time:
                    pbar.update(t)
                    time.sleep(dt)
                else:
                    pbar.close()
        elif pybullet_util.is_key_triggered(keys, '3'):

            print('=' * 80)
            print('right foot swing motion sampling')

            initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time = sample_one_step_boundary(
                lf_nominal_pos, rf_nominal_pos, 'right_foot')

            base_pos_curve, base_quat_curve, lf_pos_curve_first_half, lf_pos_curve_second_half, lf_quat_curve, rf_pos_curve_first_half, rf_pos_curve_second_half, rf_quat_curve = create_curves_from_boundary(
                initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso,
                final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso,
                final_rf_iso, mid_rf_vel, swing_time)

            pbar = tqdm(total=math.floor(swing_time))

            # base_pos_list, rf_pos_list, rf_quat_list, lf_pos_list, time_list = [], [], [], [], []
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

                joint_pos, lf_ik_success, rf_ik_success = lowerbody_inverse_kinematics(
                    base_pos, base_quat, lf_pos, lf_quat, rf_pos, rf_quat,
                    open_chain_joints_name_dict, joint_screws_in_ee_at_home,
                    ee_SE3_at_home, nominal_sensor_data_dict)

                # prepare plotting curve
                # time_list.append(t)
                # lf_pos_list.append(lf_pos)
                # rf_pos_list.append(rf_pos)
                # rf_quat_list.append(rf_quat)
                # base_pos_list.append(base_pos)

                if not (lf_ik_success and rf_ik_success):
                    print('ik fail to solve')
                    __import__('ipdb').set_trace()

                # visualize config
                pybullet_util.set_config(robot, joint_id, base_pos, base_quat,
                                         joint_pos)
                if t < swing_time:
                    pbar.update(t)
                    time.sleep(dt)
                else:
                    pbar.close()

        elif pybullet_util.is_key_triggered(keys, '4'):
            print('=' * 80)
            print('generate data set without multiprocessing')
            x_data, y_data = generate_data_set(N_SWING_MOTIONS,
                                               N_DATA_PER_SWING,
                                               lf_nominal_pos, rf_nominal_pos,
                                               nominal_sensor_data_dict,
                                               'left_foot')

        elif pybullet_util.is_key_triggered(keys, '5'):
            print('=' * 80)
            print('generate data set with multiprocessing')
            x_data_lf, y_data_lf = parallerize_data_generate(
                N_SWING_MOTIONS, N_DATA_PER_SWING, lf_nominal_pos,
                rf_nominal_pos, nominal_sensor_data_dict, 'left_foot',
                N_CPU_USE_FOR_PARALELL_COM)

            x_data_rf, y_data_rf = parallerize_data_generate(
                N_SWING_MOTIONS, N_DATA_PER_SWING, lf_nominal_pos,
                rf_nominal_pos, nominal_sensor_data_dict, 'right_foot',
                N_CPU_USE_FOR_PARALELL_COM)

            x_data = x_data_lf + x_data_rf
            y_data = y_data_lf + y_data_rf

            ##TODO: create regressor using pytorch

            # fig, axes = plt.subplots(4, 1)
            # axes[0].plot(time_list, lf_pos_list)
            # axes[1].plot(time_list, rf_pos_list)
            # axes[2].plot(time_list, base_pos_list)
            # axes[3].plot(time_list, rf_quat_list)
            # plt.show()

        # Disable forward step
        # pb.stepSimulation()

        time.sleep(dt)
        t += dt
        count += 1
