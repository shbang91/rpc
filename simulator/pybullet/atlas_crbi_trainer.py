import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import pybullet as pb
import numpy as np

from util.python_utils import interpolation
from util.python_utils import util
from util.python_utils import liegroup

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


def set_initial_config(robot, joint_id):
    # shoulder_x
    p.resetJointState(robot, joint_id["l_arm_shx"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_arm_shx"], np.pi / 4, 0.)
    # elbow_y
    p.resetJointState(robot, joint_id["l_arm_ely"], -np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_arm_ely"], np.pi / 2, 0.)
    # elbow_x
    p.resetJointState(robot, joint_id["l_arm_elx"], -np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_arm_elx"], -np.pi / 2, 0.)
    # hip_y
    p.resetJointState(robot, joint_id["l_leg_hpy"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_leg_hpy"], -np.pi / 4, 0.)
    # knee
    p.resetJointState(robot, joint_id["l_leg_kny"], np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_leg_kny"], np.pi / 2, 0.)
    # ankle
    p.resetJointState(robot, joint_id["l_leg_aky"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_leg_aky"], -np.pi / 4, 0.)


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
                                 lf_quat, open_chain_joints_name_dict,
                                 joint_screws_in_ee_at_home, ee_config_at_home,
                                 nominal_sensor_data_dict):
    """
    param: open_chain_joints_name_dict (dict): left_leg, right_leg, left_arm, right_arm
    param: joint_screws_in_ee_at_home (dict): left_leg, right_leg, left_arm, right_arm
    param: ee_config_at_home (dict): left_leg, right_leg, left_arm, right_arm
    """

    return joint_pos, lf_ik_done, rf_ik_done


#main loop
if __name__ == "__main__":

    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(cameraDistance=1.5,
                                  cameraYaw=90,
                                  cameraPitch=0,
                                  cameraTargetPosition=[0, 0, 0.8])
    ## sim physics setting
    pb.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1)
    pb.setGravity(0, 0, -9.81)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    robot = pb.loadURDF(cwd + "/robot_model/atlas/atlas.urdf",
                        Config.INITIAL_BASE_JOINT_POS,
                        Config.INITIAL_BASE_JOINT_QUAT,
                        useFixedBase=False)

    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf",
                         useFixedBase=True)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot, INITIAL_BASE_JOINT_POS, INITIAL_BASE_JOINT_QUAT,
        PRINT_ROBOT_INFO)

    ##prepare inverse kinematics
    open_chain_joints_name_dict, joint_screws_in_ee, ee_config_at_home = dict(
    ), dict(), dict()
    ee_links_name_dict, base_links_name_dict = dict(), dict()

    ########

    ##robot initial config setting
    set_initial_config(robot, joint_id)

    ##robot joint and link dynamics setting
    pybullet_util.set_joint_friction(robot, joint_id, 0)
    pybullet_util.set_link_damping(robot, link_id, 0., 0.)
