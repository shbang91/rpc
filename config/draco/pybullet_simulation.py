class Config(object):
    CONTROLLER_DT = 0.00125
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.95 - 0.21]
    #INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    # INITIAL_BASE_JOINT_POS = [0., 0., 1.5 - 0.757]
    INITIAL_BASE_JOINT_QUAT = [0., 0., 0.7071, 0.7071]

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
