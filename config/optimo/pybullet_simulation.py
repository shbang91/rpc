
def deg2rad(deg):
    return deg * 3.14159265359 / 180

class Config(object):
    CONTROLLER_DT = 0.00125
    N_SUBSTEP = 1

    INITIAL_BASE_JOINT_POS = [0, 0, 0.95 - 0.195]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    # INITIAL_JOINT_POSITION = [0.0, 3.14, 0.0, 1.57, 0.0, 1.57, 0.0]
    INITIAL_JOINT_POSITION = [deg2rad(0), deg2rad(210), 0.0, deg2rad(-120), 0.0, deg2rad(-60), 0.0]
    # INITIAL_BASE_JOINT_POS = [0., 0., 1.5 - 0.757]
    # INITIAL_BASE_JOINT_QUAT = [0., 0., 0.7071, 0.7071]

    PRINT_ROBOT_INFO = False

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50

    ##TODO:
    USE_MESHCAT = False

class OptimoLinkIdx(object):
    #  getLinkState only works for child links that are connected by a joint to the parent. It will ignore the first link index who has no parent link.
    base_link = 0
    link0 = 1
    link1 = 3
    link2 = 5
    link3 = 7
    link4 = 9
    link5 = 11
    link6 = 13
    link7 = 15
    ee = 16
    
class OptimoJointIdx(object):
    joint1 = 2
    joint2 = 4
    joint3 = 6
    joint4 = 8
    joint5 = 10
    joint6 = 12
    joint7 = 14

    

