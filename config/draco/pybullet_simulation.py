class Config(object):
    CONTROLLER_DT = 0.00125
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.95 - 0.2]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    # INITIAL_BASE_JOINT_POS = [0., 0., 1.5 - 0.757]
    # INITIAL_BASE_JOINT_QUAT = [0., 0., 0.7071, 0.7071]

    PRINT_ROBOT_INFO = False
    IP_PUB_ADDRESS = "tcp://127.0.0.1:5558"

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50

    ##TODO:
    USE_MESHCAT = False


class DracoLinkIdx(object):
    torso_link = -1
    torso_com_link = 0
    neck_pitch_link = 1
    l_multisense_camera = 2
    l_shoulder_fe_link = 3
    l_shoulder_aa_link = 4
    l_shoulder_ie_link = 5
    l_elbow_fe_link = 6
    l_wrist_ps_link = 7
    l_wrist_pitch_link = 8
    l_sake_gripper_link = 9
    l_camera = 10
    l_hand_contact = 11
    r_shoulder_fe_link = 12
    r_shoulder_aa_link = 13
    r_shoulder_ie_link = 14
    r_elbow_fe_link = 15
    r_wrist_ps_link = 16
    r_wrist_pitch_link = 17
    r_sake_gripper_link = 18
    r_camera = 19
    r_hand_contact = 20
    l_hip_ie_link = 21
    l_hip_aa_link = 22
    l_hip_fe_link = 23
    l_knee_fe_lp = 24
    l_knee_adj_link = 25
    l_knee_fe_ld = 26
    l_ankle_fe_link = 27
    l_ankle_ie_link = 28
    l_foot_contact = 29
    r_hip_ie_link = 30
    r_hip_aa_link = 31
    r_hip_fe_link = 32
    r_knee_fe_lp = 33
    r_knee_adj_link = 34
    r_knee_fe_ld = 35
    r_ankle_fe_link = 36
    r_ankle_ie_link = 37
    r_foot_contact = 38
    torso_imu = 39


class DracoJointIdx(object):
    neck_pitch = 1
    l_shoulder_fe = 3
    l_shoulder_aa = 4
    l_shoulder_ie = 5
    l_elbow_fe = 6
    l_wrist_ps = 7
    l_wrist_pitch = 8
    r_shoulder_fe = 12
    r_shoulder_aa = 13
    r_shoulder_ie = 14
    r_elbow_fe = 15
    r_wrist_ps = 16
    r_wrist_pitch = 17
    l_hip_ie = 21
    l_hip_aa = 22
    l_hip_fe = 23
    l_knee_fe_jp = 24
    l_knee_fe_jd = 26
    l_ankle_fe = 27
    l_ankle_ie = 28
    r_hip_ie = 30
    r_hip_aa = 31
    r_hip_fe = 32
    r_knee_fe_jp = 33
    r_knee_fe_jd = 35
    r_ankle_fe = 36
    r_ankle_ie = 37
