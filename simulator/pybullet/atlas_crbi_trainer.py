import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import pybullet as pb
import numpy as np

from util.python_utils import interpolation
from util.python_utils import util
from util.python_utils import liegroup

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


#main loop
if __name__ == "__main__":
    pass
