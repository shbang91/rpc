import pybullet as pb
import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

from config.go2.sim.pybullet.wbic.pybullet_params import *
from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

import signal
import shutil
import cv2

import go2_interface_py

if Config.MEASURE_COMPUTATION_TIME:
    from pytictoc import TicToc


def get_sensor_data_from_pybullet(robot):
    # follow pinocchio robotsystem urdf reading convention
    joint_pos, joint_vel = np.zeros(12), np.zeros(12)

    imu_frame_quat = np.array(pb.getLinkState(robot, Go2LinkIdx.imu, 1, 1)[1])
    
    # Front left 
    joint_pos[0] = pb.getJointState(robot, Go2JointIdx.FL_hip_joint)[0]
    joint_pos[1] = pb.getJointState(robot, Go2JointIdx.FL_thigh_joint)[0]
    joint_pos[2] = pb.getJointState(robot, Go2JointIdx.FL_calf_joint)[0]
    # Front right 
    joint_pos[3] = pb.getJointState(robot, Go2JointIdx.FR_hip_joint)[0]
    joint_pos[4] = pb.getJointState(robot, Go2JointIdx.FR_thigh_joint)[0]
    joint_pos[5] = pb.getJointState(robot, Go2JointIdx.FR_calf_joint)[0]
    # Rear left 
    joint_pos[6] = pb.getJointState(robot, Go2JointIdx.RL_hip_joint)[0]
    joint_pos[7] = pb.getJointState(robot, Go2JointIdx.RL_thigh_joint)[0]
    joint_pos[8] = pb.getJointState(robot, Go2JointIdx.RL_calf_joint)[0]
    # Rear right 
    joint_pos[9] = pb.getJointState(robot, Go2JointIdx.RR_hip_joint)[0]
    joint_pos[10] = pb.getJointState(robot, Go2JointIdx.RR_thigh_joint)[0]
    joint_pos[11] = pb.getJointState(robot, Go2JointIdx.RR_calf_joint)[0]

    imu_ang_vel = np.array(pb.getLinkState(robot, Go2LinkIdx.imu, 1, 1)[7])

    imu_dvel = pybullet_util.simulate_dVel_data(robot, Go2LinkIdx.imu,
                                                previous_torso_velocity)

    # Front left 
    joint_vel[0] = pb.getJointState(robot, Go2JointIdx.FL_hip_joint)[1]
    joint_vel[1] = pb.getJointState(robot, Go2JointIdx.FL_thigh_joint)[1]
    joint_vel[2] = pb.getJointState(robot, Go2JointIdx.FL_calf_joint)[1]
    # Front right 
    joint_vel[3] = pb.getJointState(robot, Go2JointIdx.FR_hip_joint)[1]
    joint_vel[4] = pb.getJointState(robot, Go2JointIdx.FR_thigh_joint)[1]
    joint_vel[5] = pb.getJointState(robot, Go2JointIdx.FR_calf_joint)[1]
    # Rear left 
    joint_vel[6] = pb.getJointState(robot, Go2JointIdx.RL_hip_joint)[1]
    joint_vel[7] = pb.getJointState(robot, Go2JointIdx.RL_thigh_joint)[1]
    joint_vel[8] = pb.getJointState(robot, Go2JointIdx.RL_calf_joint)[1]
    # Rear right 
    joint_vel[9] = pb.getJointState(robot, Go2JointIdx.RR_hip_joint)[1]
    joint_vel[10] = pb.getJointState(robot, Go2JointIdx.RR_thigh_joint)[1]
    joint_vel[11] = pb.getJointState(robot, Go2JointIdx.RR_calf_joint)[1]

    # normal force measured on each foot
    FL_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot, linkIndexA=Go2LinkIdx.FL_foot)
    for contact in contacts:
        # add z-component on all points of contact
        FL_normal_force += contact[9]

    FR_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot, linkIndexA=Go2LinkIdx.FR_foot)
    for contact in contacts:
        # add z-component on all points of contact
        FR_normal_force += contact[9]

    RL_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot, linkIndexA=Go2LinkIdx.RL_foot)
    for contact in contacts:
        # add z-component on all points of contact
        RL_normal_force += contact[9]

    RR_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot, linkIndexA=Go2LinkIdx.RR_foot)
    for contact in contacts:
        # add z-component on all points of contact
        RR_normal_force += contact[9]

    # Determine foot contact states based on the z-coordinate of the foot link
    b_FL_foot_contact = (True if pb.getLinkState(robot, Go2LinkIdx.FL_foot, 1, 1)[0][2] <= 0.05 else False)
    b_FR_foot_contact = (True if pb.getLinkState(robot, Go2LinkIdx.FR_foot, 1, 1)[0][2] <= 0.05 else False)
    b_RL_foot_contact = (True if pb.getLinkState(robot, Go2LinkIdx.RL_foot, 1, 1)[0][2] <= 0.05 else False)
    b_RR_foot_contact = (True if pb.getLinkState(robot, Go2LinkIdx.RR_foot, 1, 1)[0][2] <= 0.05 else False)
    
    return (imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel,
            b_FL_foot_contact, b_FR_foot_contact, b_RL_foot_contact,
            b_RR_foot_contact, FL_normal_force, FR_normal_force,
            RL_normal_force, RR_normal_force)


def apply_control_input_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL

    # Front left 
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.FL_hip_joint,
                             controlMode=mode,
                             force=command[0])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.FL_thigh_joint,
                             controlMode=mode,
                             force=command[1])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.FL_calf_joint,
                             controlMode=mode,
                             force=command[2])
    # Front right 
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.FR_hip_joint,
                             controlMode=mode,
                             force=command[3])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.FR_thigh_joint,
                             controlMode=mode,
                             force=command[4])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.FR_calf_joint,
                             controlMode=mode,
                             force=command[5])
    # Rear left 
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.RL_hip_joint,
                             controlMode=mode,
                             force=command[6])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.RL_thigh_joint,
                             controlMode=mode,
                             force=command[7])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.RL_calf_joint,
                             controlMode=mode,
                             force=command[8])
    # Rear right 
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.RR_hip_joint,
                             controlMode=mode,
                             force=command[9])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.RR_thigh_joint,
                             controlMode=mode,
                             force=command[10])
    pb.setJointMotorControl2(robot,
                             Go2JointIdx.RR_calf_joint,
                             controlMode=mode,
                             force=command[11])


def set_init_config_pybullet_robot(robot):
    knee_angle = 45
    pb.resetJointState(robot, Go2JointIdx.FL_thigh_joint,
                       np.radians(knee_angle), 0.0)
    pb.resetJointState(robot, Go2JointIdx.FL_calf_joint,
                       np.radians(-knee_angle), 0.0)

    pb.resetJointState(robot, Go2JointIdx.FR_thigh_joint,
                       np.radians(knee_angle), 0.0)
    pb.resetJointState(robot, Go2JointIdx.FR_calf_joint,
                       np.radians(-knee_angle), 0.0)

    pb.resetJointState(robot, Go2JointIdx.RL_thigh_joint,
                       np.radians(knee_angle), 0.0)
    pb.resetJointState(robot, Go2JointIdx.RL_calf_joint,
                       np.radians(-knee_angle), 0.0)

    pb.resetJointState(robot, Go2JointIdx.RR_thigh_joint,
                       np.radians(knee_angle), 0.0)
    pb.resetJointState(robot, Go2JointIdx.RR_calf_joint,
                       np.radians(-knee_angle), 0.0)


def signal_handler(signal, frame):
    if Config.MEASURE_COMPUTATION_TIME:
        print("========================================================")
        print('saving list of compuation time in "compuation_time.txt"')
        print("========================================================")
        np.savetxt("computation_time.txt",
                   np.array([compuation_cal_list]),
                   delimiter=",")

    if Config.VIDEO_RECORD:
        print("========================================================")
        print("Making Video")
        print("========================================================")
        pybullet_util.make_video(video_dir)

    pb.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    ## connect pybullet sim server
    pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    pb.resetDebugVisualizerCamera(
        cameraDistance = 1.5,
        cameraYaw = 120,
        cameraPitch = -30,
        cameraTargetPosition = [0, 0, 0.3],
    )
    ## sim physics setting
    pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                                 numSubSteps=Config.N_SUBSTEP)
    pb.setGravity(0, 0, 0)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    robot = pb.loadURDF(cwd + "/robot_model/go2/go2_description.urdf",
                        [0., 0., 0.45], [0, 0, 0, 1],
                        useFixedBase=False) 

    cylinder_robot = pb.loadURDF(
        cwd + "/robot_model/cylinder.urdf",
        Config.INITIAL_CYLINDER_BASE_JOINT_POS,
        Config.INITIAL_CYLINDER_BASE_JOINT_QUAT,
        useFixedBase=0,
    # )
    # cylinder_robot = pb.loadURDF(
    #     cwd + "/robot_model/cylinder.urdf",
    #     [1., 1., 1.], [0, 0, 0.707, 0.707],
    #     useFixedBase=False,
    )

    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf",
                         useFixedBase=1)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    (
        n_q,
        n_v,
        n_a,
        joint_id_dict,
        link_id_dict,
        pos_basejoint_to_basecom,
        rot_basejoint_to_basecom,
    ) = pybullet_util.get_robot_config(
        robot,
        Config.INITIAL_BASE_JOINT_POS,
        Config.INITIAL_BASE_JOINT_QUAT,
        Config.PRINT_ROBOT_INFO,
    )
    # robot initial config setting
    set_init_config_pybullet_robot(robot)

    # robot joint and link dynamics setting
    pybullet_util.set_joint_friction(robot, joint_id_dict, 0)
    pybullet_util.set_link_damping(robot, link_id_dict, 0.0, 0.0)

    # default robot kinematics information
    base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(robot)
    rot_world_basecom = util.quat_to_rot(np.array(base_com_quat))
    rot_world_basejoint = util.quat_to_rot(
        np.array(Config.INITIAL_BASE_JOINT_QUAT))

    pos_basejoint_to_basecom = np.dot(
        rot_world_basejoint.transpose(),
        base_com_pos - np.array(Config.INITIAL_BASE_JOINT_POS),
    )
    rot_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                      rot_world_basecom)

    # TODO: pnc interface, sensor_data, command class
    rpc_go2_interface = go2_interface_py.Go2Interface()
    rpc_go2_sensor_data = go2_interface_py.Go2SensorData()
    rpc_go2_command = go2_interface_py.Go2Command()

    # Run Simulation
    dt = Config.CONTROLLER_DT
    count = 0
    jpg_count = 0

    ## simulation options
    if Config.MEASURE_COMPUTATION_TIME:
        timer = TicToc()
        compuation_cal_list = []

    if Config.VIDEO_RECORD:
        video_dir = "video/go2"
        if os.path.exists(video_dir):
            shutil.rmtree(video_dir)
        os.makedirs(video_dir)

    ## for dvel quantity
    previous_torso_velocity = np.array([0.0, 0.0, 0.0])

    while True:
        ############################################################
        # Moving Camera Setting
        ############################################################
        base_pos, base_ori = pb.getBasePositionAndOrientation(robot)
        # pb.resetDebugVisualizerCamera(cameraDistance=1.5,
        #                               cameraYaw=120,
        #                               cameraPitch=-30,
        #                               cameraTargetPosition=base_pos +
        #                               np.array([0.5, 0.3, -base_pos[2] + 0.5]))
        ###############################################################################
        # Debugging Purpose
        ##############################################################################
        ##debugging state estimator by calculating groundtruth basejoint states
        base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(robot)
        rot_world_basecom = util.quat_to_rot(base_com_quat)
        rot_world_basejoint = np.dot(rot_world_basecom,
                                     rot_basejoint_to_basecom.transpose())
        base_joint_pos = base_com_pos - np.dot(rot_world_basejoint,
                                               pos_basejoint_to_basecom)
        base_joint_quat = util.rot_to_quat(rot_world_basejoint)

        base_com_lin_vel, base_com_ang_vel = pb.getBaseVelocity(robot)
        trans_joint_com = liegroup.RpToTrans(rot_basejoint_to_basecom,
                                             pos_basejoint_to_basecom)
        adjoint_joint_com = liegroup.Adjoint(trans_joint_com)
        twist_basecom_in_world = np.zeros(6)
        twist_basecom_in_world[0:3] = base_com_ang_vel
        twist_basecom_in_world[3:6] = base_com_lin_vel
        augrot_basecom_world = np.zeros((6, 6))
        augrot_basecom_world[0:3, 0:3] = rot_world_basecom.transpose()
        augrot_basecom_world[3:6, 3:6] = rot_world_basecom.transpose()
        twist_basecom_in_basecom = np.dot(augrot_basecom_world,
                                          twist_basecom_in_world)
        twist_basejoint_in_basejoint = np.dot(adjoint_joint_com,
                                              twist_basecom_in_basecom)
        augrot_world_basejoint = np.zeros((6, 6))
        augrot_world_basejoint[0:3, 0:3] = rot_world_basejoint
        augrot_world_basejoint[3:6, 3:6] = rot_world_basejoint
        twist_basejoint_in_world = np.dot(augrot_world_basejoint,
                                          twist_basejoint_in_basejoint)
        base_joint_ang_vel = twist_basejoint_in_world[0:3]
        base_joint_lin_vel = twist_basejoint_in_world[3:6]

        # pass debugged data to rpc interface (for ground truth estimation)
        rpc_go2_sensor_data.base_joint_pos_ = base_joint_pos
        rpc_go2_sensor_data.base_joint_quat_ = base_joint_quat
        rpc_go2_sensor_data.base_joint_lin_vel_ = base_joint_lin_vel
        rpc_go2_sensor_data.base_joint_ang_vel_ = base_joint_ang_vel

        ############################################################
        # Get Keyboard Event
        ############################################################
        keys = pb.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, "1"):
            pass

        ############################################################
        # Get Sensor Data
        ############################################################
        (imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel,
         b_FL_foot_contact, b_FR_foot_contact, b_RL_foot_contact,
         b_RR_foot_contact, FL_normal_force, FR_normal_force, RL_normal_force,
         RR_normal_force) = get_sensor_data_from_pybullet(robot)

        ## copy sensor data to rpc sensor data class
        rpc_go2_sensor_data.imu_frame_quat_ = imu_frame_quat
        rpc_go2_sensor_data.imu_ang_vel_ = imu_ang_vel
        rpc_go2_sensor_data.imu_dvel_ = imu_dvel
        rpc_go2_sensor_data.imu_lin_acc_ = imu_dvel / dt
        rpc_go2_sensor_data.joint_pos_ = joint_pos
        rpc_go2_sensor_data.joint_vel_ = joint_vel
        rpc_go2_sensor_data.b_FL_foot_contact_ = b_FL_foot_contact
        rpc_go2_sensor_data.b_FR_foot_contact_ = b_FR_foot_contact
        rpc_go2_sensor_data.b_RL_foot_contact_ = b_RL_foot_contact
        rpc_go2_sensor_data.b_RR_foot_contact_ = b_RR_foot_contact
        rpc_go2_sensor_data.FL_normal_force_ = FL_normal_force
        rpc_go2_sensor_data.FR_normal_force_ = FR_normal_force
        rpc_go2_sensor_data.RL_normal_force_ = RL_normal_force
        rpc_go2_sensor_data.RR_normal_force_ = RR_normal_force
        ############################################################
        ##compute control command
        ############################################################
        if Config.MEASURE_COMPUTATION_TIME:
            timer.tic()

        rpc_go2_interface.GetCommand(rpc_go2_sensor_data, rpc_go2_command)

        if Config.MEASURE_COMPUTATION_TIME:
            comp_time = timer.tocvalue()
            compuation_cal_list.append(comp_time)

        # copy command data from rpc command class
        rpc_trq_command = rpc_go2_command.joint_trq_cmd_
        rpc_joint_pos_command = rpc_go2_command.joint_pos_cmd_
        rpc_joint_vel_command = rpc_go2_command.joint_vel_cmd_

        # apply command to pybullet robot
        apply_control_input_to_pybullet(robot, rpc_trq_command)

        # save current torso velocity for next iteration
        previous_torso_velocity = pybullet_util.get_link_vel(
            robot, Go2LinkIdx.imu)[3:6]

        ############################################################
        # Save Image file
        ############################################################
        if (Config.VIDEO_RECORD) and (count % Config.RECORD_FREQ == 0):
            camera_data = pb.getDebugVisualizerCamera()
            frame = pybullet_util.get_camera_image_from_debug_camera(
                camera_data, Config.RENDER_WIDTH, Config.RENDER_HEIGHT)
            filename = video_dir + "/step%06d.jpg" % jpg_count
            cv2.imwrite(filename, frame)
            jpg_count += 1

        pb.stepSimulation()  # step simulation

        count += 1
