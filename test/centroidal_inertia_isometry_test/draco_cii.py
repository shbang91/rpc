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
    ## spawn draco in pybullet for visualization
    draco = pb.loadURDF(
        cwd + "/robot_model/draco/draco_modified.urdf",
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
    ) = pybullet_util.get_robot_config(draco, INITIAL_POS, INITIAL_QUAT, False)

    # pb.resetBasePositionAndOrientation(draco ,[0,0,0], [0,0,0,1])
    # print(link_id["torso_com_link"])
    # print("torso link com pose")
    # print(pb.getLinkState(draco, link_id["torso_com_link"])[0])

    nominal_sensor_data = pybullet_util.get_sensor_data(
        draco, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom
    )

    joint_pos = copy.deepcopy(nominal_sensor_data["joint_pos"])
    joint_pos["r_shoulder_aa"] = -np.pi / 2.0
    joint_pos["l_shoulder_aa"] = np.pi / 2.0

    robot_system = PinocchioRobotSystem(
        cwd + "/robot_model/draco/draco_modified.urdf",
        cwd + "/robot_model/draco",
        True,
        False,
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

    data_saver = DataSaver("draco_cii_prox.pkl")

    # for haa in np.linspace(0., 30. * np.pi / 180., num=30, endpoint=True):
    # for hfe in np.linspace(-30 * np.pi / 180., 0., num=30, endpoint=True):
    for haa in np.linspace(
        0.0 * np.pi / 180, 50.0 * np.pi / 180.0, num=50, endpoint=True
    ):
        for hfe in np.linspace(-np.pi / 3.0, 0.0, num=50, endpoint=True):
            joint_pos["r_hip_aa"] = -haa
            joint_pos["r_hip_fe"] = hfe
            joint_pos["r_knee_fe_jp"], joint_pos["r_knee_fe_jd"] = (
                -1.5 * hfe,
                -1.5 * hfe,
            )
            joint_pos["l_hip_aa"] = -haa
            joint_pos["l_hip_fe"] = hfe
            joint_pos["l_knee_fe_jp"], joint_pos["l_knee_fe_jd"] = (
                -1.5 * hfe,
                -1.5 * hfe,
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

            inertia = robot_system.Ig[0:3, 0:3]

            ## compute CII
            CII = np.linalg.det(
                np.dot(np.linalg.inv(inertia), nominal_inertia) - np.eye(3)
            )

            ### for visualization
            pb.resetJointState(draco, joint_id["r_hip_aa"], -haa)
            pb.resetJointState(draco, joint_id["r_hip_fe"], hfe)
            pb.resetJointState(draco, joint_id["r_knee_fe_jp"], -1.5 * hfe)
            pb.resetJointState(draco, joint_id["r_knee_fe_jd"], -1.5 * hfe)
            pb.resetJointState(draco, joint_id["l_hip_aa"], haa)
            pb.resetJointState(draco, joint_id["l_hip_fe"], hfe)
            pb.resetJointState(draco, joint_id["l_knee_fe_jp"], -1.5 * hfe)
            pb.resetJointState(draco, joint_id["l_knee_fe_jd"], -1.5 * hfe)
            pb.resetJointState(draco, joint_id["r_shoulder_aa"], -np.pi / 2.0)
            pb.resetJointState(draco, joint_id["l_shoulder_aa"], np.pi / 2.0)

            time.sleep(0.001)

            ## data save
            data_saver.add("cii", CII)
            data_saver.advance()
