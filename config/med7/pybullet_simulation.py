def deg2rad(deg):
    return deg * 3.14159265359 / 180


class Config(object):
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1
    INITIAL_BASE_JOINT_POS = [0, 0, 0]
    INITIAL_BASE_JOINT_QUAT = [0, 0, 0, 1]
    INITIAL_JOINT_POSITION = [-0.15, -0.78, -0.34, 1.7, 0.11, 1.135, 0.54]

    PRINT_ROBOT_INFO = True

    MEASURE_COMPUTATION_TIME = False

    VIDEO_RECORD = False
    RECORD_FREQ = 50

    VISUALIZE_ENDEFFECTOR = True  # this slows down sim speed


class Med7LinkIdx(object):
    #  getLinkState only works for child links that are connected by a joint to the parent. It will ignore the first link index who has no parent link.
    link0 = 0
    link1 = 1
    link2 = 2
    link3 = 3
    link4 = 4
    link5 = 5
    link6 = 6
    link7 = 7
    ee = 10


class Med7JointIdx(object):
    joint1 = 1
    joint2 = 2
    joint3 = 3
    joint4 = 4
    joint5 = 5
    joint6 = 6
    joint7 = 7


class ActuatorGains(object):
    # KP = [200., 200., 200., 200., 200., 200., 200.]
    # KD = [5., 5., 5., 5., 5., 5., 5.]
    KP = [0., 0., 0., 0., 0., 0., 0.]
    KD = [0., 0., 0., 0., 0., 0., 0.]
