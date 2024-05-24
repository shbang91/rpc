class Config(object):
    CONTROLLER_DT = 0.00125
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.95 - 0.195]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    INITIAL_JOINT_POSITION = [0, -2.0, 0, 0, 0, 0, 0]
    # INITIAL_BASE_JOINT_POS = [0., 0., 1.5 - 0.757]
    # INITIAL_BASE_JOINT_QUAT = [0., 0., 0.7071, 0.7071]

    PRINT_ROBOT_INFO = False

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50

    ##TODO:
    USE_MESHCAT = False

class OptimoLinkIdx(object):
    link0 = 0
    link1_passive = 1
    link2_passive = 2
    link3_passive = 3
    link4_passive = 4
    link5_passive = 5
    link6_passive = 6
    link7_passive = 7
    ee = 8
    
class OptimoJointIdx(object):
    joint1 = 0
    joint2 = 1
    joint3 = 2
    joint4 = 3
    joint5 = 4
    joint6 = 5
    joint7 = 6

    

