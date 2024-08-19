import numpy as np


class Config(object):
    CONTROLLER_DT = 0.00125
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.95 - 0.19]
    # INITIAL_BASE_JOINT_POS = [0, 0, 0.95 - 0.15]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    # INITIAL_BASE_JOINT_POS = [0., 0., 1.5 - 0.757]
    # INITIAL_BASE_JOINT_QUAT = [0., 0., 0.7071, 0.7071]
    # INITIAL_BASE_JOINT_QUAT = [0., 0., 0.383, 0.924]

    PRINT_ROBOT_INFO = True

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


class DracoManipulationLinkIdx(object):
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
    l_hand_contact = 9
    left_ezgripper_palm_link = 10
    left_ezgripper_finger_L1_1 = 11
    left_ezgripper_finger_L2_1 = 12
    left_ezgripper_finger_L1_2 = 13
    left_ezgripper_finger_L2_2 = 14
    r_shoulder_fe_link = 15
    r_shoulder_aa_link = 16
    r_shoulder_ie_link = 17
    r_elbow_fe_link = 18
    r_wrist_ps_link = 19
    r_wrist_pitch_link = 20
    r_sake_gripper_link = 21
    r_hand_contact = 22
    right_ezgripper_palm_link = 23
    right_ezgripper_finger_L1_1 = 24
    right_ezgripper_finger_L2_1 = 25
    right_ezgripper_finger_L1_2 = 26
    right_ezgripper_finger_L2_2 = 27
    l_hip_ie_link = 28
    l_hip_aa_link = 29
    l_hip_fe_link = 30
    l_knee_fe_lp = 31
    l_knee_adj_link = 32
    l_knee_fe_ld = 33
    l_ankle_fe_link = 34
    l_ankle_ie_link = 35
    l_foot_contact = 36
    r_hip_ie_link = 37
    r_hip_aa_link = 38
    r_hip_fe_link = 39
    r_knee_fe_lp = 40
    r_knee_adj_link = 41
    r_knee_fe_ld = 42
    r_ankle_fe_link = 43
    r_ankle_ie_link = 44
    r_foot_contact = 45
    torso_imu = 46


class DracoManipulationJointIdx(object):
    neck_pitch = 1
    l_shoulder_fe = 2
    l_shoulder_aa = 3
    l_shoulder_ie = 4
    l_elbow_fe = 5
    l_wrist_ps = 6
    l_wrist_pitch = 7
    r_shoulder_fe = 15
    r_shoulder_aa = 16
    r_shoulder_ie = 17
    r_elbow_fe = 18
    r_wrist_ps = 19
    r_wrist_pitch = 20
    l_hip_ie = 28
    l_hip_aa = 29
    l_hip_fe = 30
    l_knee_fe_jp = 31
    l_knee_fe_jd = 33
    l_ankle_fe = 34
    l_ankle_ie = 35
    r_hip_ie = 37
    r_hip_aa = 38
    r_hip_fe = 39
    r_knee_fe_jp = 40
    r_knee_fe_jd = 42
    r_ankle_fe = 43
    r_ankle_ie = 44


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
