import base64

import pybullet as pb
import time
import os
from PIL import Image
cwd = os.getcwd()
import sys
import io
from PIL import Image

sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  #include pybind module

import time

import numpy as np
import ipdb

from config.draco.pybullet_simulation import *
from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

# from util.python_utils.pybullet_camera_util import Camera
sys.path.append(os.getcwd() + '/build')
import base64
import zmq
from messages.multisense_pb2 import *

import copy

import signal
import shutil
import cv2

from loop_rate_limiters import RateLimiter

import draco_interface_py

if Config.MEASURE_COMPUTATION_TIME:
    from pytictoc import TicToc

# Simulated noise characteristics
imu_dvel_bias = np.array([0.0, 0.0, 0.0])
l_contact_volt_noise = 0.001
r_contact_volt_noise = 0.001

import math

# create publisher of camera data (from pybullet)
context = zmq.Context()
camera_socket = context.socket(zmq.PUB)
camera_socket.bind(Config.IP_PUB_ADDRESS)
camera_pb_msg = camera_msg()

dcamera_socket = context.socket(zmq.PUB)
dcamera_socket.bind("tcp://127.0.0.1:5559")
# print(Config.IP_PUB_ADDRESS)

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def get_sensor_data_from_pybullet(robot):

    #follow pinocchio robotsystem urdf reading convention
    joint_pos, joint_vel = np.zeros(27), np.zeros(27)

    imu_frame_quat = np.array(
        pb.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[1])
    #LF
    joint_pos[0] = pb.getJointState(robot, DracoJointIdx.l_hip_ie)[0]
    joint_pos[1] = pb.getJointState(robot, DracoJointIdx.l_hip_aa)[0]
    joint_pos[2] = pb.getJointState(robot, DracoJointIdx.l_hip_fe)[0]
    joint_pos[3] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[0]
    joint_pos[4] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[0]
    joint_pos[5] = pb.getJointState(robot, DracoJointIdx.l_ankle_fe)[0]
    joint_pos[6] = pb.getJointState(robot, DracoJointIdx.l_ankle_ie)[0]
    #LH
    joint_pos[7] = pb.getJointState(robot, DracoJointIdx.l_shoulder_fe)[0]
    joint_pos[8] = pb.getJointState(robot, DracoJointIdx.l_shoulder_aa)[0]
    joint_pos[9] = pb.getJointState(robot, DracoJointIdx.l_shoulder_ie)[0]
    joint_pos[10] = pb.getJointState(robot, DracoJointIdx.l_elbow_fe)[0]
    joint_pos[11] = pb.getJointState(robot, DracoJointIdx.l_wrist_ps)[0]
    joint_pos[12] = pb.getJointState(robot, DracoJointIdx.l_wrist_pitch)[0]
    #neck
    joint_pos[13] = pb.getJointState(robot, DracoJointIdx.neck_pitch)[0]
    #RF
    joint_pos[14] = pb.getJointState(robot, DracoJointIdx.r_hip_ie)[0]
    joint_pos[15] = pb.getJointState(robot, DracoJointIdx.r_hip_aa)[0]
    joint_pos[16] = pb.getJointState(robot, DracoJointIdx.r_hip_fe)[0]
    joint_pos[17] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[0]
    joint_pos[18] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[0]
    joint_pos[19] = pb.getJointState(robot, DracoJointIdx.r_ankle_fe)[0]
    joint_pos[20] = pb.getJointState(robot, DracoJointIdx.r_ankle_ie)[0]
    #RH
    joint_pos[21] = pb.getJointState(robot, DracoJointIdx.r_shoulder_fe)[0]
    joint_pos[22] = pb.getJointState(robot, DracoJointIdx.r_shoulder_aa)[0]
    joint_pos[23] = pb.getJointState(robot, DracoJointIdx.r_shoulder_ie)[0]
    joint_pos[24] = pb.getJointState(robot, DracoJointIdx.r_elbow_fe)[0]
    joint_pos[25] = pb.getJointState(robot, DracoJointIdx.r_wrist_ps)[0]
    joint_pos[26] = pb.getJointState(robot, DracoJointIdx.r_wrist_pitch)[0]

    imu_ang_vel = np.array(
        pb.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[7])

    imu_dvel = pybullet_util.simulate_dVel_data(robot, link_id_dict,
                                                previous_torso_velocity)

    #LF
    joint_vel[0] = pb.getJointState(robot, DracoJointIdx.l_hip_ie)[1]
    joint_vel[1] = pb.getJointState(robot, DracoJointIdx.l_hip_aa)[1]
    joint_vel[2] = pb.getJointState(robot, DracoJointIdx.l_hip_fe)[1]
    joint_vel[3] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[1]
    joint_vel[4] = pb.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[1]
    joint_vel[5] = pb.getJointState(robot, DracoJointIdx.l_ankle_fe)[1]
    joint_vel[6] = pb.getJointState(robot, DracoJointIdx.l_ankle_ie)[1]
    #LH
    joint_vel[7] = pb.getJointState(robot, DracoJointIdx.l_shoulder_fe)[1]
    joint_vel[8] = pb.getJointState(robot, DracoJointIdx.l_shoulder_aa)[1]
    joint_vel[9] = pb.getJointState(robot, DracoJointIdx.l_shoulder_ie)[1]
    joint_vel[10] = pb.getJointState(robot, DracoJointIdx.l_elbow_fe)[1]
    joint_vel[11] = pb.getJointState(robot, DracoJointIdx.l_wrist_ps)[1]
    joint_vel[12] = pb.getJointState(robot, DracoJointIdx.l_wrist_pitch)[1]
    #neck
    joint_vel[13] = pb.getJointState(robot, DracoJointIdx.neck_pitch)[1]
    #RF
    joint_vel[14] = pb.getJointState(robot, DracoJointIdx.r_hip_ie)[1]
    joint_vel[15] = pb.getJointState(robot, DracoJointIdx.r_hip_aa)[1]
    joint_vel[16] = pb.getJointState(robot, DracoJointIdx.r_hip_fe)[1]
    joint_vel[17] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[1]
    joint_vel[18] = pb.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[1]
    joint_vel[19] = pb.getJointState(robot, DracoJointIdx.r_ankle_fe)[1]
    joint_vel[20] = pb.getJointState(robot, DracoJointIdx.r_ankle_ie)[1]
    #RH
    joint_vel[21] = pb.getJointState(robot, DracoJointIdx.r_shoulder_fe)[1]
    joint_vel[22] = pb.getJointState(robot, DracoJointIdx.r_shoulder_aa)[1]
    joint_vel[23] = pb.getJointState(robot, DracoJointIdx.r_shoulder_ie)[1]
    joint_vel[24] = pb.getJointState(robot, DracoJointIdx.r_elbow_fe)[1]
    joint_vel[25] = pb.getJointState(robot, DracoJointIdx.r_wrist_ps)[1]
    joint_vel[26] = pb.getJointState(robot, DracoJointIdx.r_wrist_pitch)[1]

    # normal force measured on each foot
    _l_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot, linkIndexA=DracoLinkIdx.l_ankle_ie_link)
    for contact in contacts:
        # add z-component on all points of contact
        _l_normal_force += contact[9]

    _r_normal_force = 0
    contacts = pb.getContactPoints(bodyA=robot, linkIndexA=DracoLinkIdx.r_ankle_ie_link)
    for contact in contacts:
        # add z-component on all points of contact
        _r_normal_force += contact[9]

    b_lf_contact = True if pb.getLinkState(robot, DracoLinkIdx.l_foot_contact,
                                           1, 1)[0][2] <= 0.05 else False
    b_rf_contact = True if pb.getLinkState(robot, DracoLinkIdx.r_foot_contact,
                                           1, 1)[0][2] <= 0.05 else False
    return imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, \
        b_rf_contact, _l_normal_force, _r_normal_force


#TODO:try to modify with setjointmotorcontrol "array" API
def apply_control_input_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL

    #LF
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_ie,
                             controlMode=mode,
                             force=command[0])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_aa,
                             controlMode=mode,
                             force=command[1])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_fe,
                             controlMode=mode,
                             force=command[2])
    # pb.setJointMotorControl2(robot, DracoJointIdx.l_knee_fe_jp, controlMode=mode, force=command[3])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_knee_fe_jd,
                             controlMode=mode,
                             force=command[4])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_ankle_fe,
                             controlMode=mode,
                             force=command[5])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_ankle_ie,
                             controlMode=mode,
                             force=command[6])

    #LH
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_fe,
                             controlMode=mode,
                             force=command[7])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_aa,
                             controlMode=mode,
                             force=command[8])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_ie,
                             controlMode=mode,
                             force=command[9])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_elbow_fe,
                             controlMode=mode,
                             force=command[10])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_wrist_ps,
                             controlMode=mode,
                             force=command[11])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.l_wrist_pitch,
                             controlMode=mode,
                             force=command[12])

    #neck
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.neck_pitch,
                             controlMode=mode,
                             force=command[13])

    #RF
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_ie,
                             controlMode=mode,
                             force=command[14])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_aa,
                             controlMode=mode,
                             force=command[15])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_fe,
                             controlMode=mode,
                             force=command[16])
    # pb.setJointMotorControl2(robot, DracoJointIdx.r_knee_fe_jd, controlMode=mode, force=command[17])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_knee_fe_jd,
                             controlMode=mode,
                             force=command[18])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_ankle_fe,
                             controlMode=mode,
                             force=command[19])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_ankle_ie,
                             controlMode=mode,
                             force=command[20])

    #RH
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_fe,
                             controlMode=mode,
                             force=command[21])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_aa,
                             controlMode=mode,
                             force=command[22])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_ie,
                             controlMode=mode,
                             force=command[23])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_elbow_fe,
                             controlMode=mode,
                             force=command[24])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_wrist_ps,
                             controlMode=mode,
                             force=command[25])
    pb.setJointMotorControl2(robot,
                             DracoJointIdx.r_wrist_pitch,
                             controlMode=mode,
                             force=command[26])


def set_init_config_pybullet_robot(robot):
    # Upperbody
    pb.resetJointState(robot, DracoJointIdx.l_shoulder_aa, np.pi / 6, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_elbow_fe, -np.pi / 2, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_shoulder_aa, -np.pi / 6, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_elbow_fe, -np.pi / 2, 0.)

    # Lowerbody
    hip_yaw_angle = 0
    pb.resetJointState(robot, DracoJointIdx.l_hip_aa,
                       np.radians(hip_yaw_angle), 0.)
    pb.resetJointState(robot, DracoJointIdx.l_hip_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_knee_fe_jp, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_knee_fe_jd, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_ankle_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.l_ankle_ie,
                       np.radians(-hip_yaw_angle), 0.)

    pb.resetJointState(robot, DracoJointIdx.r_hip_aa,
                       np.radians(-hip_yaw_angle), 0.)
    pb.resetJointState(robot, DracoJointIdx.r_hip_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_knee_fe_jp, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_knee_fe_jd, np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_ankle_fe, -np.pi / 4, 0.)
    pb.resetJointState(robot, DracoJointIdx.r_ankle_ie,
                       np.radians(hip_yaw_angle), 0.)


def signal_handler(signal, frame):
    # if Config.VIDEO_RECORD:
    # pybullet_util.make_video(video_dir, False)
    if Config.MEASURE_COMPUTATION_TIME:
        print('========================================================')
        print('saving list of compuation time in "compuation_time.txt"')
        print('========================================================')
        np.savetxt('computation_time.txt',
                   np.array([compuation_cal_list]),
                   delimiter=',')

    if Config.VIDEO_RECORD:
        print('========================================================')
        print('Making Video')
        print('========================================================')
        pybullet_util.make_video(video_dir)

    pb.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":

    ## connect pybullet sim server
    pb.connect(pb.GUI)

    pb.resetDebugVisualizerCamera(cameraDistance=1.5,
                                  cameraYaw=120,
                                  cameraPitch=-30,
                                  cameraTargetPosition=[0, 0, 0.5])
    ## sim physics setting
    pb.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                                 numSubSteps=Config.N_SUBSTEP)
    pb.setGravity(0, 0, -9.81)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    # draco_humanoid = pb.loadURDF(cwd + "/robot_model/draco/draco_modified.urdf",
    # Config.INITIAL_BASE_JOINT_POS,
    # Config.INITIAL_BASE_JOINT_QUAT,
    # useFixedBase=0)
    draco_humanoid = pb.loadURDF(cwd +
                                 "/robot_model/draco/draco_modified.urdf",
                                 Config.INITIAL_BASE_JOINT_POS,
                                 Config.INITIAL_BASE_JOINT_QUAT,
                                 useFixedBase=0)

    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf",
                         useFixedBase=1)
    door = pb.loadURDF(cwd + "/robot_model/ground/navy_door.urdf",
                         [1.5, 0., 0.02],
                         # [0., 0., 0.0, 0.0],
                         [0., 0., 0.7071068, 0.7071068],
                         # [0., 0., 0.8660254, 0.5],
                         useFixedBase=1)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    #TODO:modify this function without dictionary container
    n_q, n_v, n_a, joint_id_dict, link_id_dict, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        draco_humanoid, Config.INITIAL_BASE_JOINT_POS,
        Config.INITIAL_BASE_JOINT_QUAT, Config.PRINT_ROBOT_INFO)

    #robot initial config setting
    set_init_config_pybullet_robot(draco_humanoid)

    #robot joint and link dynamics setting
    #TODO:modify this function without dictionary container
    pybullet_util.set_joint_friction(draco_humanoid, joint_id_dict, 0)
    pybullet_util.set_link_damping(draco_humanoid, link_id_dict, 0., 0.)

    ## rolling contact joint constraint
    c1 = pb.createConstraint(draco_humanoid,
                             DracoLinkIdx.l_knee_fe_lp,
                             draco_humanoid,
                             DracoLinkIdx.l_knee_fe_ld,
                             jointType=pb.JOINT_GEAR,
                             jointAxis=[0, 1, 0],
                             parentFramePosition=[0, 0, 0],
                             childFramePosition=[0, 0, 0])
    pb.changeConstraint(c1, gearRatio=-1, maxForce=500, erp=10)

    c2 = pb.createConstraint(draco_humanoid,
                             DracoLinkIdx.r_knee_fe_lp,
                             draco_humanoid,
                             DracoLinkIdx.r_knee_fe_ld,
                             jointType=pb.JOINT_GEAR,
                             jointAxis=[0, 1, 0],
                             parentFramePosition=[0, 0, 0],
                             childFramePosition=[0, 0, 0])
    pb.changeConstraint(c2, gearRatio=-1, maxForce=500, erp=10)

    #pnc interface, sensor_data, command class
    rpc_draco_interface = draco_interface_py.DracoInterface()
    rpc_draco_sensor_data = draco_interface_py.DracoSensorData()
    rpc_draco_command = draco_interface_py.DracoCommand()

    #TODO
    #active jointidx list in sequence
    active_jointidx_list = []  #for setjointmotorcontrolarray

    #default robot kinematics information
    base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(
        draco_humanoid)

    rot_world_basecom = util.quat_to_rot(np.array(base_com_quat))
    rot_world_basejoint = util.quat_to_rot(
        np.array(Config.INITIAL_BASE_JOINT_QUAT))

    pos_basejoint_to_basecom = np.dot(
        rot_world_basejoint.transpose(),
        base_com_pos - np.array(Config.INITIAL_BASE_JOINT_POS))
    rot_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                      rot_world_basecom)

    # Run Simulation
    dt = Config.CONTROLLER_DT
    count = 0
    jpg_count = 0

    ## simulation options
    if Config.MEASURE_COMPUTATION_TIME:
        timer = TicToc()
        compuation_cal_list = []

    if Config.VIDEO_RECORD:
        video_dir = 'video/draco'
        if os.path.exists(video_dir):
            shutil.rmtree(video_dir)
        os.makedirs(video_dir)

    previous_torso_velocity = np.array([0., 0., 0.])
    # rate = RateLimiter(frequency=1. / dt)
    imu_dvel_bias = np.array([0.0, 0.0, 0.0])


    # rgbim.save('test_img/rgbtest'+str(counter)+'.png')
    # depim.save('test_img/depth'+str(counter)+'.tiff')
    # rate = RateLimiter(frequency=1. / dt)
    counter = 0

    def get_point_cloud(width, height, view_matrix, proj_matrix):
        # based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer

        # get a depth image
        # "infinite" depths will have a value close to 1
        image_arr = pb.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
        depth = image_arr[3]
        print('dbrpg')
        print(image_arr[2].shape)
        print(depth.shape)
        imgs = image_arr[2][:, :, [2, 1, 0]]
        print(imgs.shape)

        # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
        proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
        view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
        tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

        # create a grid with pixel coordinates and depth values
        y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
        y *= -1.
        x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
        h = np.ones_like(z)

        pixels = np.stack([x, y, z, h], axis=1)
        # filter out "infinite" depths
        print("shape of pixels",pixels.shape)

        print("where jey ha",(pixels[:,2] < 0.99).shape)
        print("where jey ha",(pixels[:,2] < 0.99) + 0)
        mask = (pixels[:,2] < 0.99) + 0
        mask = mask.reshape(height,width)
        print(mask.shape)

        pixels = pixels[z < 0.99]
        print("shape of pixels",pixels.shape)
        pixels[:, 2] = 2 * pixels[:, 2] - 1

        # turn pixels to world coordinates
        points = np.matmul(tran_pix_world, pixels.T).T
        points /= points[:, 3: 4]
        points = points[:, :3]

        rgb = 0

        return points, rgb

    while (True):
        l_normal_volt_noise = np.random.normal(0, l_contact_volt_noise)
        r_normal_volt_noise = np.random.normal(0, r_contact_volt_noise)

        counter+=1

        if counter % 1000 ==0:
            (x,y,z), (a,b,c,d),loc1,local2,world1,world2 = pb.getLinkState(draco_humanoid,DracoLinkIdx.l_multisense_camera)
            # print("this is abcd", a,b,c,d)
            # print("this is w2", world2)
            (rol,pit,yaw) = euler_from_quaternion(a,b,c,d)

            xA, yA, zA = x, y, z
            zA = zA + 0.3 # make the camera a little higher than the robot
            distance = 10
            # compute focusing point of the camera
            xB = xA + math.cos(yaw) * distance
            yB = yA + math.sin(yaw) * distance
            zB = zA

            view_matrix = pb.computeViewMatrix(
                cameraEyePosition=[xA, yA, zA],
                cameraTargetPosition=[xB, yB, zB],
                cameraUpVector=[0, 0, 1.0]
            )

            print(xA,yA,zA)
            print(xB,yB,zB)


            #
            # view_matrix = pb.computeViewMatrixFromYawPitchRoll(
            #     cameraTargetPosition=[xB,yB,zB],
            #     distance=10,
            #     yaw=yaw,
            #     pitch=pit,
            #     roll=rol,
            #     upAxisIndex=2
            #
            # )
            far = 250
            near = 0.02
            H = 1920
            W = 1080
            projection_matrix = pb.computeProjectionMatrixFOV(
                fov=120, aspect=1.5, nearVal=near, farVal=far)

            aimgs = pb.getCameraImage(H, W,
                                      view_matrix,
                                      projection_matrix, shadow=True,
                                      renderer=pb.ER_BULLET_HARDWARE_OPENGL)
            imgs = aimgs[2]

            dimgs = aimgs[3]
            depim = Image.fromarray(dimgs)

            depth = far * near / (far - (far-near) * dimgs)
            print('this is depth', sum(sum(depth)))
            # print(depth.shape)
            a, b = get_point_cloud(H,W,view_matrix,projection_matrix)

            # offset point cloud
            # lense_offset = np.array([0., 0., 0.])
            # camera_base_offset = np.array([-0.0025, 0, 0.352]) + lense_offset
            # base_est_pos = rpc_draco_sensor_data.base_joint_pos_
            # camera_est_pos = base_est_pos + camera_base_offset
            # # a -= camera_est_pos

            print("pcd shape", a.shape)
            # print(type(a))
            # print(dimgs)
            # print(depim.shape)
            imgs = imgs[:, :, [2, 1, 0]]
            print(imgs.shape)

            # print(imgs.shape)

            encoded, buffer = cv2.imencode('.jpg', imgs)
            encoded2, buffer2 = cv2.imencode('.jpg', dimgs)
            strencond = buffer.tobytes()
            f4 = io.BytesIO(strencond)
            f5 = io.BufferedReader(f4)
            bytes_as_np_array = np.frombuffer(buffer, dtype=np.uint8)
            # print(bytes_as_np_array.shape)

            # camera_socket.send_string(base64.b64encode(buffer))
            # camera_socket.send_string(str(base64.b64encode(buffer)))

            # f = open("/Users/timsm1/Desktop/test_img/rgbtest10.jpg",'rb')
            # print('check it f', type(f))
            # print('check it', type(f5))
            # print('length f', len(bytearray(f5.read())))
            bytes = bytearray(f5.read())
            # print('check it', type(bytes))
            strng = base64.b64encode(bytes)
            # print('length bytes',len(bytes))
            # print(bytes)
            # print(type(strng))
            # print(type(a.tobytes()))
            # print(type(np.frombuffer(a.tobytes())))
            whack = np.frombuffer(a.tobytes())
            whack = whack.reshape((int(len(whack)/3),3))
            # print(len(whack))
            # print(whack.shape)
            # print(whack)
            # print(a)
            camera_socket.send(strng)

            dstrencond = buffer2.tobytes()
            df4 = io.BytesIO(dstrencond)
            df5 = io.BufferedReader(df4)
            dbytes = bytearray(df5.read())
            # print('check it', type(bytes))
            dstrng = base64.b64encode(dbytes)
            # dcamera_socket.send(dstrng)
            print('this a', a.shape)
            dcamera_socket.send(a)

            filename = "/Users/timsm1/Desktop/test_img/calo"+str(counter)+'.jpg'
            f = open(filename, 'wb')

            hoda = bytearray(base64.b64decode(strng))
            f.write(hoda)
            f.close()
            # print('bada')


        # print('bado',len(base64.b64encode(buffer)))
        # print('as',type(base64.b64encode(buffer)))
        #
        # print('bada',len(str(base64.b64encode(buffer))))
        # print('as',type(str(base64.b64encode(buffer))))
        #
        # print('as',(base64.b64encode(buffer)))
        #
        # print('as',len(bytes(str(base64.b64encode(buffer)), 'UTF-8')))
        #
        # base64.b64decode(bytes(str(base64.b64encode(buffer)), 'UTF-8'))


    # print(type(base64.b64encode(buffer)))
        # print('hodabada')
        # print(np.fromstring(base64.b64decode(base64.b64encode(buffer)),dtype=np.uint8).shape)
        # daba =np.fromstring(base64.b64decode(base64.b64encode(buffer)),dtype=np.uint8)
        # print(cv2.imdecode(daba,cv2.IMREAD_COLOR).shape)

        # if counter % 10 == 0:
        #     filename = '/Users/timsm1/Desktop/test_img/rgbtest'+str(counter)+'.jpg'
        #     cv2.imwrite(filename, imgs)

        # (rol,pit,yaw) = euler_from_quaternion(a,b,c,d)
        # print("rpy",rol,pit,yaw)
        #
        # frame = pybullet_util.get_camera_image([x, y, z], 0.15, 45,
        #                                        0, 0, 60., 1920, 1080,
        #                                        0.1, 100.)
        # frame = frame[:, :, [2, 1, 0]]  # << RGB to BGR
        # if counter % 10 == 0:
        #     filename = '/Users/timsm1/Desktop/test_img/rgbtest'+str(counter)+'.jpg'
        #     cv2.imwrite(filename, frame)
        #     # rgbim.save('/Users/timsm1/Desktop/test_img/rgbtest'+str(counter)+'.png')


        # filename = video_dir + '/step%06d.jpg' % jpg_count
        # cv2.imwrite(filename, frame)

        # rgb_img, depth_img, seg_img = cam.get_pybullet_image()
        # pb.resetDebugVisualizerCamera(cameraDistance=0.5,
        #                               cameraYaw=0,
        #                               cameraPitch=0,
        #
        #                               cameraTargetPosition=[x, y, z])
        # # print(x,y,z)
        # counter += 1
        # if counter % 100 == 0:
        #     img = pb.getCameraImage(300, 300, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
        #     print('this is working')
        #     rgbBuffer = img[2]
        #     # depthBuffer = img[3] # .astype(np.uint8) ?
        #     rgbim = Image.fromarray(rgbBuffer)
        #     rgbim.save('/Users/timsm1/Desktop/test_img/rgbtest'+str(counter)+'.png')

        # depim = Image.fromarray(depthBuffer)
        ###############################################################################
        #Debugging Purpose
        ##############################################################################
        ##debugging state estimator by calculating groundtruth basejoint states
        base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(
            draco_humanoid)
        rot_world_basecom = util.quat_to_rot(base_com_quat)
        rot_world_basejoint = np.dot(rot_world_basecom,
                                     rot_basejoint_to_basecom.transpose())
        base_joint_pos = base_com_pos - np.dot(rot_world_basejoint,
                                               pos_basejoint_to_basecom)
        base_joint_quat = util.rot_to_quat(rot_world_basejoint)

        base_com_lin_vel, base_com_ang_vel = pb.getBaseVelocity(draco_humanoid)
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

        #pass debugged data to rpc interface
        rpc_draco_sensor_data.base_joint_pos_ = base_joint_pos
        rpc_draco_sensor_data.base_joint_quat_ = base_joint_quat
        rpc_draco_sensor_data.base_joint_lin_vel_ = base_joint_lin_vel
        rpc_draco_sensor_data.base_joint_ang_vel_ = base_joint_ang_vel
        ##############################################################################
        ##############################################################################

        # Get Keyboard Event
        keys = pb.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, '1'):
            rpc_draco_interface.interrupt_.PressOne()
        elif pybullet_util.is_key_triggered(keys, '2'):
            rpc_draco_interface.interrupt_.PressTwo()
        elif pybullet_util.is_key_triggered(keys, '4'):
            rpc_draco_interface.interrupt_.PressFour()
        elif pybullet_util.is_key_triggered(keys, '5'):
            rpc_draco_interface.interrupt_.PressFive()
        elif pybullet_util.is_key_triggered(keys, '6'):
            rpc_draco_interface.interrupt_.PressSix()
        elif pybullet_util.is_key_triggered(keys, '7'):
            rpc_draco_interface.interrupt_.PressSeven()
        elif pybullet_util.is_key_triggered(keys, '8'):
            rpc_draco_interface.interrupt_.PressEight()
        elif pybullet_util.is_key_triggered(keys, '9'):
            rpc_draco_interface.interrupt_.PressNine()

        #get sensor data
        imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, b_rf_contact, \
            l_normal_force, r_normal_force = get_sensor_data_from_pybullet(
            draco_humanoid)
        l_normal_force = pybullet_util.simulate_contact_sensor(l_normal_force)
        r_normal_force = pybullet_util.simulate_contact_sensor(r_normal_force)
        imu_dvel = pybullet_util.add_sensor_noise(imu_dvel, imu_dvel_bias)
        l_normal_force = pybullet_util.add_sensor_noise(l_normal_force, l_normal_volt_noise)
        r_normal_force = pybullet_util.add_sensor_noise(r_normal_force, r_normal_volt_noise)

        #copy sensor data to rpc sensor data class
        rpc_draco_sensor_data.imu_frame_quat_ = imu_frame_quat
        rpc_draco_sensor_data.imu_ang_vel_ = imu_ang_vel
        rpc_draco_sensor_data.imu_dvel_ = imu_dvel
        rpc_draco_sensor_data.joint_pos_ = joint_pos
        rpc_draco_sensor_data.joint_vel_ = joint_vel
        rpc_draco_sensor_data.b_lf_contact_ = b_lf_contact
        rpc_draco_sensor_data.b_rf_contact_ = b_rf_contact
        rpc_draco_sensor_data.lf_contact_normal_ = l_normal_force
        rpc_draco_sensor_data.rf_contact_normal_ = r_normal_force

        ##Debugging

        ##compute control command
        if Config.MEASURE_COMPUTATION_TIME:
            timer.tic()

        rpc_draco_interface.GetCommand(rpc_draco_sensor_data,
                                       rpc_draco_command)

        if Config.MEASURE_COMPUTATION_TIME:
            comp_time = timer.tocvalue()
            compuation_cal_list.append(comp_time)

        #copy command data from rpc command class
        rpc_trq_command = rpc_draco_command.joint_trq_cmd_
        rpc_joint_pos_command = rpc_draco_command.joint_pos_cmd_
        rpc_joint_vel_command = rpc_draco_command.joint_vel_cmd_

        #apply command to pybullet robot
        apply_control_input_to_pybullet(draco_humanoid, rpc_trq_command)

        # lfoot_pos = pybullet_util.get_link_iso(draco_humanoid,
        # save current torso velocity for next iteration
        previous_torso_velocity = pybullet_util.get_link_vel(
            draco_humanoid, link_id_dict['torso_imu'])[3:6]

        # DracoLinkIdx.l_foot_contact)[0:3, 3]
        # rfoot_pos = pybullet_util.get_link_iso(draco_humanoid,
        # DracoLinkIdx.r_foot_contact)[0:3, 3]
        # print("------------------------------------")
        # print(rfoot_pos[1] - lfoot_pos[1])

        # print("trq command printout")
        # print(rpc_trq_command)
        # print("jpos command printout")
        # print(rpc_joint_pos_command)
        # print("jpos command printout")
        # print(rpc_joint_vel_command)

        # Save Image file
        if (Config.VIDEO_RECORD) and (count % Config.RECORD_FREQ == 0):
            frame = pybullet_util.get_camera_image([1., 0.5, 1.], 1.0, 120,
                                                   -15, 0, 60., 1920, 1080,
                                                   0.1, 100.)
            frame = frame[:, :, [2, 1, 0]]  # << RGB to BGR
            filename = video_dir + '/step%06d.jpg' % jpg_count
            cv2.imwrite(filename, frame)
            jpg_count += 1

        pb.stepSimulation()  #step simulation
        # rate.sleep()  # while loop rate limiter

        count += 1
