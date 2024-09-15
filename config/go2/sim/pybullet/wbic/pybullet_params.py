import numpy as np


class Config(object):
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.35]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]

    PRINT_ROBOT_INFO = True

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50
    RENDER_WIDTH = 1920
    RENDER_HEIGHT = 1080

    ##TODO:
    USE_MESHCAT = False


class Go2LinkIdx(object):
    base = -1
    Head_upper = 0
    Head_lower = 1
    FL_hip = 2
    FL_thigh = 3
    FL_calf = 4
    FL_calflower = 5
    FL_calflower1 = 6
    FL_foot = 7
    FR_hip = 8
    FR_thigh = 9
    FR_calf = 10
    FR_calflower = 11
    FR_calflower1 = 12
    FR_foot = 13
    RL_hip = 14
    RL_thigh = 15
    RL_calf = 16
    RL_calflower = 17
    RL_calflower1 = 18
    RL_foot = 19
    RR_hip = 20
    RR_thigh = 21
    RR_calf = 22
    RR_calflower = 23
    RR_calflower1 = 24
    RR_foot = 25
    imu = 26
    radar = 27


class Go2JointIdx(object):
    FL_hip_joint = 2
    FL_thigh_joint = 3
    FL_calf_joint = 4
    FR_hip_joint = 8
    FR_thigh_joint = 9
    FR_calf_joint = 10
    RL_hip_joint = 14
    RL_thigh_joint = 15
    RL_calf_joint = 16
    RR_hip_joint = 20
    RR_thigh_joint = 21
    RR_calf_joint = 22


class JointGains(object):
    # kp = 5. * np.ones(27)
    # kd = 0. * np.ones(27)
    kp = np.array([
        10, 10, 10, 10, 10, 10, 10, 5, 5, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10,
        10, 10, 5, 5, 5, 5, 5, 5
    ])
    kd = np.array([
        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001,
        0.001, 0.001, 0.01, 0.01, 0.001, 0.01, 0.01, 0.01, 0.01, 0.01, 0.001,
        0.001, 0.001, 0.001, 0.001, 0.001
    ])
