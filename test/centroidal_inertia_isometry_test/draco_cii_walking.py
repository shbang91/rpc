import os
import sys
import copy
import pybullet as pb
import numpy as np

from util.python_utils import pybullet_util
from util.python_utils import interpolation
from util.python_utils import util
from util.python_utils import liegroup
from util.python_utils import robot_kinematics

from util.python_utils.pinocchio_robot_system import PinocchioRobotSystem

from plot.data_saver import DataSaver

cwd = os.getcwd()
sys.path.append(cwd)

## parameters
INITIAL_POS = [0.0, 0.0, 0.74]
INITIAL_QUAT = [0.0, 0.0, 0.0, 1.0]


def set_initial_config(robot, joint_id):
    # pb.resetJointState(robot, joint_id["l_knee_adj"], np.pi / 2, 0.)
    # pb.resetJointState(robot, joint_id["r_knee_adj"], np.pi / 2, 0.)

    pb.resetJointState(robot, joint_id["l_knee_fe_jp"], np.pi / 4, 0.0)
    pb.resetJointState(robot, joint_id["r_knee_fe_jp"], np.pi / 4, 0.0)
    pb.resetJointState(robot, joint_id["l_knee_fe_jd"], np.pi / 4, 0.0)
    pb.resetJointState(robot, joint_id["r_knee_fe_jd"], np.pi / 4, 0.0)

    pb.resetJointState(robot, joint_id["l_hip_fe"], -np.pi / 4, 0.0)
    pb.resetJointState(robot, joint_id["r_hip_fe"], -np.pi / 4, 0.0)

    pb.resetJointState(robot, joint_id["l_ankle_fe"], -np.pi / 4, 0.0)
    pb.resetJointState(robot, joint_id["r_ankle_fe"], -np.pi / 4, 0.0)

    pb.resetJointState(robot, joint_id["l_shoulder_aa"], np.pi / 2, 0.0)
    pb.resetJointState(robot, joint_id["r_shoulder_aa"], -np.pi / 2, 0.0)


if __name__ == "__main__":
    # pybullet GUI for visualization
    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=120,
        cameraPitch=-15,
        cameraTargetPosition=[0, 0, 1.0],
    )

    ## spawn draco in pybullet
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    # robot = pb.loadURDF(cwd + '/robot_model/draco/draco_ik.urdf',
    robot = pb.loadURDF(
        cwd + "/robot_model/draco/draco_modified_cii.urdf",
        INITIAL_POS,
        INITIAL_QUAT,
        useFixedBase=False,
    )
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    ## robot config
    (
        nq,
        nv,
        na,
        joint_id,
        link_id,
        pos_basejoint_to_basecom,
        rot_basejoint_to_basecom,
    ) = pybullet_util.get_robot_config(robot, INITIAL_POS, INITIAL_QUAT, False)

    ## ik prep
    joint_screws_in_ee_at_home, ee_SE3_at_home = dict(), dict()
    open_chain_joints_name_dict, ee_links_name_dict, base_links_name_dict = (
        dict(),
        dict(),
        dict(),
    )

    leg_list = ["left_leg", "right_leg"]

    # open_chain_joints_name_dict['left_leg'] = [
    # 'l_hip_ie', 'l_hip_aa', 'l_hip_fe', 'l_knee_adj', 'l_ankle_fe',
    # 'l_ankle_ie'
    # ]
    # open_chain_joints_name_dict['right_leg'] = [
    # 'r_hip_ie', 'r_hip_aa', 'r_hip_fe', 'r_knee_adj', 'r_ankle_fe',
    # 'r_ankle_ie'
    # ]

    open_chain_joints_name_dict["left_leg"] = [
        "l_hip_ie",
        "l_hip_aa",
        "l_hip_fe",
        "l_knee_fe_jp",
        "l_knee_fe_jd",
        "l_ankle_fe",
        "l_ankle_ie",
    ]
    open_chain_joints_name_dict["right_leg"] = [
        "r_hip_ie",
        "r_hip_aa",
        "r_hip_fe",
        "r_knee_fe_jp",
        "r_knee_fe_jd",
        "r_ankle_fe",
        "r_ankle_ie",
    ]

    ee_links_name_dict["left_leg"] = "l_foot_contact"
    ee_links_name_dict["right_leg"] = "r_foot_contact"

    base_links_name = "torso_link"

    for leg in leg_list:
        joint_screws_in_ee_at_home[leg], ee_SE3_at_home[leg] = (
            robot_kinematics.get_kinematics_config(
                robot,
                joint_id,
                link_id,
                open_chain_joints_name_dict[leg],
                base_links_name,
                ee_links_name_dict[leg],
            )
        )

    leg_length = (
        pybullet_util.get_link_iso(robot, link_id["l_hip_ie_link"])[2, 3]
        - pybullet_util.get_link_iso(robot, link_id["r_foot_contact"])[2, 3]
    )
    # print("==========leg_length===========")
    # print(leg_length)
    # __import__('ipdb').set_trace()

    ## set initial joint pos
    set_initial_config(robot, joint_id)

    ## get initial sensor data
    nominal_sensor_data = pybullet_util.get_sensor_data(
        robot, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom
    )

    nominal_joint_pos = copy.deepcopy(nominal_sensor_data["joint_pos"])
    initial_joint_pos = copy.deepcopy(nominal_sensor_data["joint_pos"])
    nominal_base_pos = np.copy(nominal_sensor_data["base_com_pos"])
    nominal_base_quat = np.copy(nominal_sensor_data["base_com_quat"])
    nominal_base_iso = liegroup.RpToTrans(
        util.quat_to_rot(nominal_base_quat), nominal_base_pos
    )
    nominal_rf_iso = np.copy(
        pybullet_util.get_link_iso(robot, link_id["r_foot_contact"])
    )
    nominal_lf_iso = np.copy(
        pybullet_util.get_link_iso(robot, link_id["l_foot_contact"])
    )

    ## update virtual robot model
    robot_system = PinocchioRobotSystem(
        # cwd + '/robot_model/draco/draco_ik.urdf', cwd + '/robot_model/draco',
        cwd + "/robot_model/draco/draco_modified_cii.urdf",
        cwd + "/robot_model/draco",
        False,
        False,
    )
    robot_system.update_system(
        nominal_sensor_data["base_joint_pos"],
        nominal_sensor_data["base_joint_quat"],
        nominal_sensor_data["base_joint_lin_vel"],
        nominal_sensor_data["base_joint_ang_vel"],
        nominal_joint_pos,
        nominal_sensor_data["joint_vel"],
        True,
    )

    ## calculate rotational CCRBI
    nominal_inertia = robot_system.Ig[0:3, 0:3]

    ## data saver for RCCRBI
    data_saver = DataSaver("draco_cii_collocated_walking_traj.pkl")

    ## create operational space walking trajectories with parameters
    ## parameters: Foot SE(3), Swing height, Swing time, number of node

    ## motion boundary
    SWING_TIME_LB, SWING_TIME_UB = 0.5, 1.0
    SWING_HEIGHT_LB, SWING_HEIGHT_UB = 0.1, 0.2

    RFOOT_POS_DEV_LB = np.array([-0.1, -0.1, -0.05])
    RFOOT_POS_DEV_UB = np.array([0.1, 0.1, 0.05])

    FOOT_EA_LB = np.array([np.deg2rad(-5.0), np.deg2rad(-15.0), np.deg2rad(-45.0)])
    FOOT_EA_UB = np.array([np.deg2rad(5.0), np.deg2rad(15.0), np.deg2rad(45.0)])

    ## create swing foot trajectories
    ## sample one step boundary

    # swing_time = np.random.uniform(SWING_TIME_LB, SWING_TIME_UB)
    # SWING_HEIGHT = np.random.uniform(SWING_HEIGHT_LB, SWING_HEIGHT_UB)

    # final_rf_pos = nominal_rf_iso[:3, 3] + np.random.uniform(
    # RFOOT_POS_DEV_LB, RFOOT_POS_DEV_UB)
    # final_rf_ea = np.random.uniform(FOOT_EA_LB, FOOT_EA_UB)

    swing_time = 0.5
    ONE_STEP_X_LB, ONE_STEP_X_UB = 0.1, 0.2
    ONE_STEP_Y_LB, ONE_STEP_Y_UB = -0.1, 0.1
    SWING_HEIGHT = 0.05
    NUM_NODE_PER_SWING = 30

    for one_step_x in np.linspace(ONE_STEP_X_LB, ONE_STEP_X_UB, num=10):
        for one_step_y in np.linspace(ONE_STEP_Y_LB, ONE_STEP_Y_UB, num=10):
            final_rf_pos = nominal_rf_iso[:3, 3] + np.array([one_step_x, one_step_y, 0])
            final_rf_ea = np.array([0.0, 0.0, 0.0])
            final_rf_rot = util.euler_to_rot(final_rf_ea)
            final_rf_iso = liegroup.RpToTrans(final_rf_rot, final_rf_pos)

            mid_rf_iso = interpolation.iso_interpolate(
                nominal_rf_iso, final_rf_iso, 0.5
            )
            mid_rf_iso[2, 3] += SWING_HEIGHT
            mid_rf_vel = (final_rf_iso[:3, 3] - nominal_rf_iso[:3, 3]) / swing_time

            mid_rf_vel[2] = 0.0

            final_base_iso = interpolation.iso_interpolate(
                nominal_lf_iso, final_rf_iso, 0.5
            )
            final_base_iso[2, 3] = nominal_base_pos[2]

            #### Create curves using one step boundary
            base_pos_curve = interpolation.HermiteCurveVec(
                nominal_base_iso[:3, 3],
                np.zeros(3),
                final_base_iso[:3, 3],
                np.zeros(3),
                swing_time,
            )
            base_quat_curve = interpolation.HermiteCurveQuat(
                util.rot_to_quat(nominal_base_iso[:3, :3]),
                np.zeros(3),
                util.rot_to_quat(final_base_iso[:3, :3]),
                np.zeros(3),
                swing_time,
            )

            rf_pos_curve_first_half = interpolation.HermiteCurveVec(
                nominal_rf_iso[:3, 3],
                np.zeros(3),
                mid_rf_iso[:3, 3],
                mid_rf_vel,
                swing_time / 2.0,
            )
            rf_pos_curve_second_half = interpolation.HermiteCurveVec(
                mid_rf_iso[:3, 3],
                mid_rf_vel,
                final_rf_iso[:3, 3],
                np.zeros(3),
                swing_time / 2.0,
            )

            rf_quat_curve = interpolation.HermiteCurveQuat(
                util.rot_to_quat(nominal_rf_iso[:3, :3]),
                np.zeros(3),
                util.rot_to_quat(final_rf_iso[:3, :3]),
                np.zeros(3),
                swing_time,
            )

            #### prepare IK
            for t in np.linspace(0.0, swing_time, num=NUM_NODE_PER_SWING):
                base_pos = base_pos_curve.evaluate(t)
                base_quat = base_quat_curve.evaluate(t)

                if t <= swing_time / 2.0:
                    rf_pos = rf_pos_curve_first_half.evaluate(t)
                else:
                    rf_pos = rf_pos_curve_second_half.evaluate(t - swing_time / 2.0)

                rf_quat = rf_quat_curve.evaluate(t)

                lf_pos = nominal_lf_iso[:3, 3]
                lf_quat = util.rot_to_quat(nominal_lf_iso[:3, :3])

                #### calculate IK
                #### right foot
                T_w_base = liegroup.RpToTrans(util.quat_to_rot(base_quat), base_pos)

                T_w_rf = liegroup.RpToTrans(util.quat_to_rot(rf_quat), rf_pos)
                des_T_base_rf = np.dot(liegroup.TransInv(T_w_base), T_w_rf)

                rf_initial_guess = np.array(
                    [
                        initial_joint_pos[joint_name]
                        for joint_name in open_chain_joints_name_dict["right_leg"]
                    ]
                )

                rf_joint_pos_sol, rf_ik_success = robot_kinematics.IKinBody(
                    joint_screws_in_ee_at_home["right_leg"],
                    ee_SE3_at_home["right_leg"],
                    des_T_base_rf,
                    rf_initial_guess,
                )

                #### left foot
                T_w_lf = liegroup.RpToTrans(util.quat_to_rot(lf_quat), lf_pos)
                des_T_base_lf = np.dot(liegroup.TransInv(T_w_base), T_w_lf)

                lf_initial_guess = np.array(
                    [
                        nominal_joint_pos[joint_name]
                        for joint_name in open_chain_joints_name_dict["left_leg"]
                    ]
                )

                lf_joint_pos_sol, lf_ik_success = robot_kinematics.IKinBody(
                    joint_screws_in_ee_at_home["left_leg"],
                    ee_SE3_at_home["left_leg"],
                    des_T_base_lf,
                    lf_initial_guess,
                )

                # lf_initial_guess = lf_joint_pos_sol

                if not rf_ik_success or not lf_ik_success:
                    raise Exception("ik does not find the satisfying solution")

                #### set config in pybullet
                for i, joint_name in enumerate(
                    open_chain_joints_name_dict["right_leg"]
                ):
                    nominal_joint_pos[joint_name] = rf_joint_pos_sol[i]
                for i, joint_name in enumerate(open_chain_joints_name_dict["left_leg"]):
                    nominal_joint_pos[joint_name] = lf_joint_pos_sol[i]

                pybullet_util.set_config(
                    robot, joint_id, base_pos, base_quat, nominal_joint_pos
                )

                ## calculate inertia
                robot_system.update_system(
                    base_pos,
                    base_quat,
                    nominal_sensor_data["base_joint_lin_vel"],
                    nominal_sensor_data["base_joint_ang_vel"],
                    nominal_joint_pos,
                    nominal_sensor_data["joint_vel"],
                    True,
                )

                inertia = robot_system.Ig[0:3, 0:3]

                ## compute CII
                CII = np.linalg.det(
                    np.dot(np.linalg.inv(inertia), nominal_inertia) - np.eye(3)
                )

                data_saver.add("cii", CII)
                data_saver.advance()

    print("=================================================")
    print("Done collecting Data")
    print("=================================================")
