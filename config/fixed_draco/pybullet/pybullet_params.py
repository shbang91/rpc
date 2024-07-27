class Config(object):
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 1]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]

    PRINT_ROBOT_INFO = False

    KP, KD = dict(), dict()

    KP["neck_pitch"] = 20.
    KD["neck_pitch"] = 2.

    KP["r_hip_ie"] = 300
    KD["r_hip_ie"] = 2
    KP["l_hip_ie"] = 300
    KD["l_hip_ie"] = 2

    KP["r_hip_aa"] = 300
    KD["r_hip_aa"] = 2
    KP["l_hip_aa"] = 300
    KD["l_hip_aa"] = 2

    KP["r_hip_fe"] = 400
    KD["r_hip_fe"] = 2
    KP["l_hip_fe"] = 400
    KD["l_hip_fe"] = 2

    KP["r_knee_fe_jd"] = 400
    KD["r_knee_fe_jd"] = 2
    KP["l_knee_fe_jd"] = 400
    KD["l_knee_fe_jd"] = 2

    KP["r_ankle_fe"] = 150
    KD["r_ankle_fe"] = 2
    KP["l_ankle_fe"] = 150.
    KD["l_ankle_fe"] = 2

    KP["r_ankle_ie"] = 150.
    KD["r_ankle_ie"] = 2
    KP["l_ankle_ie"] = 150.
    KD["l_ankle_ie"] = 2

    KP["r_shoulder_fe"] = 200.
    KD["r_shoulder_fe"] = 2.
    KP["l_shoulder_fe"] = 200.
    KD["l_shoulder_fe"] = 2.

    KP["r_shoulder_aa"] = 200.
    KD["r_shoulder_aa"] = 2.
    KP["l_shoulder_aa"] = 200.
    KD["l_shoulder_aa"] = 2.

    KP["r_shoulder_ie"] = 200.
    KD["r_shoulder_ie"] = 2.
    KP["l_shoulder_ie"] = 200.
    KD["l_shoulder_ie"] = 2.

    KP["r_elbow_fe"] = 200.
    KD["r_elbow_fe"] = 2
    KP["l_elbow_fe"] = 200.
    KD["l_elbow_fe"] = 2

    KP["r_wrist_ps"] = 200.
    KD["r_wrist_ps"] = 2
    KP["l_wrist_ps"] = 200.
    KD["l_wrist_ps"] = 2

    KP["r_wrist_pitch"] = 200.
    KD["r_wrist_pitch"] = 2
    KP["l_wrist_pitch"] = 200.
    KD["l_wrist_pitch"] = 2
