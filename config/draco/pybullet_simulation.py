class Config(object):
    #CONTROLLER_DT = 0.00125
    #CONTROLLER_DT = 0.0025
    CONTROLLER_DT = 0.00175
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.95 - 0.195]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    # INITIAL_BASE_JOINT_POS = [0., 0., 1.5 - 0.757]
    # INITIAL_BASE_JOINT_QUAT = [0., 0., 0.7071, 0.7071]

    PRINT_ROBOT_INFO = False

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50

    ##TODO:
    USE_MESHCAT = False


class DracoLinkIdx(object):
    torso_link = -1
    torso_com_link = 0
    neck_pitch_link = 1
    l_shoulder_fe_link = 2
    l_shoulder_aa_link = 3
    l_shoulder_ie_link = 4
    l_elbow_fe_link = 5
    l_wrist_ps_link = 6
    l_wrist_pitch_link = 7
    l_sake_gripper_link = 8
    l_camera = 9
    l_hand_contact = 10
    r_shoulder_fe_link = 11
    r_shoulder_aa_link = 12
    r_shoulder_ie_link = 13
    r_elbow_fe_link = 14
    r_wrist_ps_link = 15
    r_wrist_pitch_link = 16
    r_sake_gripper_link = 17
    r_camera = 18
    r_hand_contact = 19
    l_hip_ie_link = 20
    l_hip_aa_link = 21
    l_hip_fe_link = 22
    l_knee_fe_lp = 23
    l_knee_adj_link = 24
    l_knee_fe_ld = 25
    l_ankle_fe_link = 26
    l_ankle_ie_link = 27
    l_foot_contact = 28
    r_hip_ie_link = 29
    r_hip_aa_link = 30
    r_hip_fe_link = 31
    r_knee_fe_lp = 32
    r_knee_adj_link = 33
    r_knee_fe_ld = 34
    r_ankle_fe_link = 35
    r_ankle_ie_link = 36
    r_foot_contact = 37
    torso_imu = 38


class DracoJointIdx(object):
    neck_pitch = 1
    l_shoulder_fe = 2
    l_shoulder_aa = 3
    l_shoulder_ie = 4
    l_elbow_fe = 5
    l_wrist_ps = 6
    l_wrist_pitch = 7
    r_shoulder_fe = 11
    r_shoulder_aa = 12
    r_shoulder_ie = 13
    r_elbow_fe = 14
    r_wrist_ps = 15
    r_wrist_pitch = 16
    l_hip_ie = 20
    l_hip_aa = 21
    l_hip_fe = 22
    l_knee_fe_jp = 23
    l_knee_fe_jd = 25
    l_ankle_fe = 26
    l_ankle_ie = 27
    r_hip_ie = 29
    r_hip_aa = 30
    r_hip_fe = 31
    r_knee_fe_jp = 32
    r_knee_fe_jd = 34
    r_ankle_fe = 35
    r_ankle_ie = 36


class AlipParams(object):
    N_BATCH = 1
    TS = 0.275
    NT_qp = 4
    NT_mpc = 1
    NS = 4
    MASS = 35.
    ZH = 0.69
    WIDTH = 0.05
    G = 9.81
    UFP_X_MAX = 0.55
    UFP_Y_MAX = 0.4
    UFP_Y_MIN = 0.04
    UFP_Y_MIN_turn = 0.15
    LX_OFFSET = 0.
    LY_DES = 0.
    COM_YAW = 0.
    INITIAL_STANCE_LEG = -1
    SWING_HEIGHT = 0.05

    PX = 1.
    PY = 1.
    PLX = 1.
    PLY = 1.
    PBOUND = 5.

    RF_Z_MAX = 1000.0

    #COM
    """
    UCOM_X_MAX = 0.3
    UCOM_Y_EXT = 0.3
    UCOM_Y_INT = 0.1

    """
    #Mixed
    """
    ALPHA = 0.7

    """
