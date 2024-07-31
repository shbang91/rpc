import os
import sys
import copy
import time
import pybullet as pb
import numpy as np

from util.python_utils import pybullet_util
from util.python_utils.pinocchio_robot_system import PinocchioRobotSystem

from plot.data_saver import DataSaver

cwd = os.getcwd()
sys.path.append(cwd)

INITIAL_POS = [0.0, 0.0, 1.0]
INITIAL_QUAT = [0.0, 0.0, 0.0, 1.0]

if __name__ == "__main__":
    # pybullet env
    pb.connect(pb.GUI)
    pb.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=120,
        cameraPitch=-15,
        cameraTargetPosition=[0, 0, 1.0],
    )
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    ## spawn atlas in pybullet for visualization
    atlas = pb.loadURDF(
        cwd + "/robot_model/atlas/atlas.urdf",
        INITIAL_POS,
        INITIAL_QUAT,
        useFixedBase=True,
    )
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    (
        nq,
        nv,
        na,
        joint_id,
        link_id,
        pos_basejoint_to_basecom,
        rot_basejoint_to_basecom,
    ) = pybullet_util.get_robot_config(atlas, INITIAL_POS, INITIAL_QUAT, False)

    nominal_sensor_data = pybullet_util.get_sensor_data(
        atlas, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom
    )

    joint_pos = copy.deepcopy(nominal_sensor_data["joint_pos"])
    # joint_pos['r_arm_shx'] = np.pi / 2
    # joint_pos['l_arm_shx'] = -np.pi / 2

    robot_system = PinocchioRobotSystem(
        cwd + "/robot_model/atlas/atlas.urdf", cwd + "/robot_model/atlas", True, False
    )

    robot_system.update_system(
        nominal_sensor_data["base_joint_pos"],
        nominal_sensor_data["base_joint_quat"],
        nominal_sensor_data["base_joint_lin_vel"],
        nominal_sensor_data["base_joint_ang_vel"],
        joint_pos,
        nominal_sensor_data["joint_vel"],
        True,
    )

    nominal_inertia = robot_system.Ig[0:3, 0:3]

    data_saver = DataSaver("atlas_cii.pkl")

    # for haa in np.linspace(0., 30. * np.pi / 180., num=30, endpoint=True):
    # for hfe in np.linspace(-np.pi / 6., 0., num=30, endpoint=True):
    for haa in np.linspace(
        -15.0 * np.pi / 180, 50.0 * np.pi / 180.0, num=50, endpoint=True
    ):
        for hfe in np.linspace(-np.pi / 3.0, 0.0, num=50, endpoint=True):
            joint_pos["r_leg_hpx"] = -haa
            joint_pos["r_leg_hpy"] = hfe
            joint_pos["r_leg_kny"] = -2 * hfe

            joint_pos["l_leg_hpx"] = haa
            joint_pos["l_leg_hpy"] = hfe
            joint_pos["l_leg_kny"] = -2 * hfe

            robot_system.update_system(
                nominal_sensor_data["base_joint_pos"],
                nominal_sensor_data["base_joint_quat"],
                nominal_sensor_data["base_joint_lin_vel"],
                nominal_sensor_data["base_joint_ang_vel"],
                joint_pos,
                nominal_sensor_data["joint_vel"],
                True,
            )

            inertia = robot_system.Ig[0:3, 0:3]

            # compute CII
            CII = np.linalg.det(
                np.dot(np.linalg.inv(inertia), nominal_inertia) - np.eye(3)
            )

            # for visualization
            # pb.resetJointState(atlas, joint_id['r_arm_shx'], np.pi / 2)
            # pb.resetJointState(atlas, joint_id['l_arm_shx'], -np.pi / 2)
            pb.resetJointState(atlas, joint_id["r_leg_hpx"], -haa)
            pb.resetJointState(atlas, joint_id["r_leg_hpy"], hfe)
            pb.resetJointState(atlas, joint_id["r_leg_kny"], -2 * hfe)
            pb.resetJointState(atlas, joint_id["l_leg_hpx"], haa)
            pb.resetJointState(atlas, joint_id["l_leg_hpy"], hfe)
            pb.resetJointState(atlas, joint_id["l_leg_kny"], -2 * hfe)

            time.sleep(0.001)

            # data save
            data_saver.add("cii", CII)
            data_saver.advance()
