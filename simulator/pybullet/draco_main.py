import pybullet as pb
import time
import os

cwd = os.getcwd()
import sys

sys.path.append(cwd)
import time

import numpy as np

from config.draco.pybullet_simulation import Config
from util.python_utils import pybullet_util


def set_initial_config(robot, joint_id):
    # Upperbody
    pb.resetJointState(robot, joint_id["l_shoulder_aa"], np.pi / 6, 0.)
    pb.resetJointState(robot, joint_id["l_elbow_fe"], -np.pi / 2, 0.)
    pb.resetJointState(robot, joint_id["r_shoulder_aa"], -np.pi / 6, 0.)
    pb.resetJointState(robot, joint_id["r_elbow_fe"], -np.pi / 2, 0.)

    # Lowerbody
    hip_yaw_angle = 5
    pb.resetJointState(robot, joint_id["l_hip_aa"], np.radians(hip_yaw_angle),
                       0.)
    pb.resetJointState(robot, joint_id["l_hip_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_knee_fe_jp"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_knee_fe_jd"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_ankle_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["l_ankle_ie"],
                       np.radians(-hip_yaw_angle), 0.)

    pb.resetJointState(robot, joint_id["r_hip_aa"], np.radians(-hip_yaw_angle),
                       0.)
    pb.resetJointState(robot, joint_id["r_hip_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_knee_fe_jp"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_knee_fe_jd"], np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_ankle_fe"], -np.pi / 4, 0.)
    pb.resetJointState(robot, joint_id["r_ankle_ie"],
                       np.radians(hip_yaw_angle), 0.)


pb.connect(pb.GUI)

## sim physics setting
pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                             numSubSteps=Config.N_SUBSTEP)
pb.setGravity(0, 0, -9.81)

## robot spawn & initial kinematics and dynamics setting
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
draco_humanoid = pb.loadURDF(cwd + "/robot_model/draco/draco.urdf",
                             Config.INITIAL_BASE_JOINT_POS,
                             Config.INITIAL_BASE_JOINT_QUAT)

ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=1)
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
    draco_humanoid, Config.INITIAL_BASE_JOINT_POS,
    Config.INITIAL_BASE_JOINT_QUAT, Config.PRINT_ROBOT_INFO)

#robot initial config setting
set_initial_config(draco_humanoid, joint_id)

#robot joint and link dynamics setting
pybullet_util.set_joint_friction(draco_humanoid, joint_id, 0)
pybullet_util.set_link_damping(draco_humanoid, link_id, 0., 0.)

## rolling contact joint constraint
c = pb.createConstraint(draco_humanoid,
                        link_id['l_knee_fe_lp'],
                        draco_humanoid,
                        link_id['l_knee_fe_ld'],
                        jointType=pb.JOINT_GEAR,
                        jointAxis=[0, 1, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0, 0, 0])
pb.changeConstraint(c, gearRatio=-1, maxForce=500, erp=10)

c = pb.createConstraint(draco_humanoid,
                        link_id['r_knee_fe_lp'],
                        draco_humanoid,
                        link_id['r_knee_fe_ld'],
                        jointType=pb.JOINT_GEAR,
                        jointAxis=[0, 1, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0, 0, 0])
pb.changeConstraint(c, gearRatio=-1, maxForce=500, erp=10)

#TODO:
#interface, sensor_data, command class

## Run Simulation
t = 0
dt = Config.CONTROLLER_DT
count = 0

while (1):

    ##get_sensor_data
    sensor_data = pybullet_util.get_sensor_data(draco_humanoid, joint_id,
                                                link_id,
                                                pos_basejoint_to_basecom,
                                                rot_basejoint_to_basecom)
    ##compute control command

    ##apply motor torque

    #step simulation
    pb.stepSimulation()
    # time.sleep(dt)  ## Is it necessary?

    t += dt
    count += 1
