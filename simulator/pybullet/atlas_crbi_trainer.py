import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import pybullet as pb
import numpy as np

import time
import copy

from util.python_utils import pybullet_util
from util.python_utils import interpolation
from util.python_utils import util
from util.python_utils import liegroup
from util.python_utils import robot_kinematics

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
N_DATA_PER_SWING = 15


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


def sample_locomotion_boundary(nominal_lf_iso, nominal_rf_iso,
                               swing_foot_side):
    swing_time = np.random.uniform(SWING_TIME_LB, SWING_TIME_UB)
    swing_height = np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

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
        initial_lf_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        initial_lf_rot = util.euler_to_rot(initial_lf_ea)
        final_lf_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)
        final_lf_rot = util.euler_to_rot(final_lf_ea)

        initial_lf_iso = liegroup.RpToTrans(initial_lf_rot, initial_lf_pos)
        final_lf_iso = liegroup.RpToTrans(final_lf_rot, final_lf_pos)
        mid_lf_iso = interpolation.iso_interpolate(initial_lf_iso,
                                                   final_lf_iso, 0.5)
        mid_lf_iso[2, 3] = swing_height
        mid_lf_vel = (final_lf_iso[0:3, 3] -
                      initial_lf_iso[0:3, 3]) / swing_time
        mid_lf_vel[2] = 0.

        #sample base
        initial_base_iso = interpolation.iso_interpolate(
            initial_rf_iso, initial_lf_iso, 0.5)
        initial_base_iso[2, 3] = np.random.uniform(BASE_HEIGHT_LB,
                                                   BASE_HEIGHT_UB)

        final_base_iso = interpolation.iso_interpolate(final_rf_iso,
                                                       final_lf_iso, 0.5)
        final_base_iso[2, 3] = np.random.uniform(BASE_HEIGHT_LB,
                                                 BASE_HEIGHT_UB)

    elif swing_foot_side == "right_foot":
        pass
    else:
        raise ValueError

    return initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time


def generate_locomotion_trajectories(initial_base_iso, final_base_iso,
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
        swing_time)

    lf_pos_curve_second_half = interpolation.HermiteCurveVec(
        mid_lf_iso[0:3, 3], mid_lf_vel, final_lf_iso[0:3, 3], np.zeros(3),
        swing_time)

    lf_quat_curve = interpolation.HermiteCurveQuat(
        util.rot_to_quat(initial_lf_iso[0:3, 0:3]), np.zeros(3),
        util.rot_to_quat(final_lf_iso[0:3, 0:3]), np.zeros(3), swing_time)

    rf_pos_curve_first_half = interpolation.HermiteCurveVec(
        initial_rf_iso[0:3, 3], np.zeros(3), mid_rf_iso[0:3, 3], mid_rf_vel,
        swing_time)

    rf_pos_curve_second_half = interpolation.HermiteCurveVec(
        mid_rf_iso[0:3, 3], mid_rf_vel, final_rf_iso[0:3, 3], np.zeros(3),
        swing_time)

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
        nominal_sensor_data_dict['joint_pos'][joint_name]
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
        nominal_sensor_data_dict['joint_pos'][joint_name]
        for joint_name in open_chain_joints_name_dict['right_leg']
    ])
    rf_joint_pos_sol, rf_ik_success = robot_kinematics.IKinBody(
        joint_screws_in_ee_at_home['right_leg'], ee_SE3_at_home['right_leg'],
        des_T_base_rf, rf_initial_guess)
    for i, joint_name in enumerate(open_chain_joints_name_dict['right_leg']):
        joint_pos[joint_name] = rf_joint_pos_sol[i]

    return joint_pos, lf_ik_success, rf_ik_success


#main loop
if __name__ == "__main__":

    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=1.5,
                                  cameraYaw=120,
                                  cameraPitch=-30,
                                  cameraTargetPosition=[1, 0.5, 1.5])
    ## sim physics setting
    pb.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1)
    pb.setGravity(0, 0, -9.81)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    robot = pb.loadURDF(cwd + "/robot_model/atlas/atlas.urdf",
                        INITIAL_POS_WORLD_TO_BASEJOINT,
                        INITIAL_QUAT_WORLD_TO_BASEJOINT,
                        useFixedBase=False)

    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf",
                         useFixedBase=True)
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

    ##sim
    t = 0.
    dt = DT
    count = 0

    while (True):

        # Get Keyboard Event
        keys = pb.getKeyboardEvents()

        if pybullet_util.is_key_triggered(keys, '8'):

            initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso, final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso, final_rf_iso, mid_rf_vel, swing_time = sample_locomotion_boundary(
                lf_nominal_pos, rf_nominal_pos, 'left_foot')

            base_pos_curve, base_quat_curve, lf_pos_curve_first_half, lf_pos_curve_second_half, lf_quat_curve, rf_pos_curve_first_half, rf_pos_curve_second_half, rf_quat_curve = generate_locomotion_trajectories(
                initial_base_iso, final_base_iso, initial_lf_iso, mid_lf_iso,
                final_lf_iso, mid_lf_vel, initial_rf_iso, mid_rf_iso,
                final_rf_iso, mid_rf_vel, swing_time)

            for t in np.linspace(0, swing_time, num=N_DATA_PER_SWING):
                base_pos = base_pos_curve.evaluate(t)
                base_quat = base_quat_curve.evaluate(t)

                if t <= swing_time / 2.:
                    lf_pos = lf_pos_curve_first_half.evaluate(t)
                    rf_pos = rf_pos_curve_first_half.evaluate(t)
                else:
                    lf_pos = lf_pos_curve_second_half.evaluate(t)
                    rf_pos = rf_pos_curve_second_half.evaluate(t)

                lf_quat = lf_quat_curve.evaluate(t)
                rf_quat = rf_quat_curve.evaluate(t)

                joint_pos, lf_ik_success, rf_ik_success = lowerbody_inverse_kinematics(
                    base_pos, base_quat, lf_pos, lf_quat, rf_pos, lf_quat,
                    open_chain_joints_name_dict, joint_screws_in_ee_at_home,
                    ee_SE3_at_home, nominal_sensor_data_dict)

                # visualize config
                pybullet_util.set_config(robot, joint_id, base_pos, base_quat,
                                         joint_pos)

                __import__('ipdb').set_trace()

        # Disable forward step
        # pb.stepSimulation()

        time.sleep(dt)
        t += dt
        count += 1
