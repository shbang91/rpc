import os
import sys
from collections import OrderedDict

import pybullet as pb
import numpy as np

np.set_printoptions(precision=5)
from tqdm import tqdm
import cv2
import imageio

from util.python_utils import util
from util.python_utils import liegroup


def get_robot_config(robot,
                     initial_pos=None,
                     initial_quat=None,
                     b_print_info=False, client = None):
    nq, nv, na, joint_id, link_id = 0, 0, 0, OrderedDict(), OrderedDict()
    link_id[(client.getBodyInfo(robot)[0]).decode("utf-8")] = -1
    for i in range(client.getNumJoints(robot)):
        info = client.getJointInfo(robot, i)
        if info[2] != client.JOINT_FIXED:
            joint_id[info[1].decode("utf-8")] = info[0]
        link_id[info[12].decode("utf-8")] = info[0]
        nq = max(nq, info[3])
        nv = max(nv, info[4])
    nq += 1
    nv += 1
    na = len(joint_id)

    base_pos, base_quat = client.getBasePositionAndOrientation(robot)
    rot_world_com = util.quat_to_rot(base_quat)
    initial_pos = [0., 0., 0.] if initial_pos is None else initial_pos
    initial_quat = [0., 0., 0., 1.] if initial_quat is None else initial_quat
    rot_world_basejoint = util.quat_to_rot(np.array(initial_quat))
    pos_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                      base_pos - np.array(initial_pos))
    rot_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                      rot_world_com)

    if b_print_info:
        print("=" * 80)
        print("SimulationRobot")
        print("nq: ", nq, ", nv: ", nv, ", na: ", na)
        print("Vector from base joint frame to base com frame")
        print(pos_basejoint_to_basecom)
        print("Rotation from base joint frame to base com frame")
        print(rot_basejoint_to_basecom)
        print("+" * 80)
        print("Joint Infos")
        util.pretty_print(joint_id)
        print("+" * 80)
        print("Link Infos")
        util.pretty_print(link_id)
        print("base com pos")
        print(base_pos)
        print("base com ori")
        print(rot_world_com)
        print("base joint pos")
        print(initial_pos)
        print("base joint ori")
        print(rot_world_basejoint)
        print(client.getJointP)

    return nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom


def get_kinematics_config(robot, joint_id, link_id, open_chain_joints,
                          base_link, ee_link, client):
    joint_screws_in_ee = np.zeros((6, len(open_chain_joints)))
    ee_link_state = client.getLinkState(robot, link_id[ee_link], 1, 1)
    if link_id[base_link] == -1:
        base_pos, base_quat = client.getBasePositionAndOrientation(robot)
    else:
        base_link_state = client.getLinkState(robot, link_id[base_link], 1, 1)
        base_pos, base_quat = base_link_state[0], base_link_state[1]
    T_w_b = liegroup.RpToTrans(util.quat_to_rot(np.array(base_quat)),
                               np.array(base_pos))
    T_w_ee = liegroup.RpToTrans(util.quat_to_rot(np.array(ee_link_state[1])),
                                np.array(ee_link_state[0]))
    T_b_ee = np.dot(liegroup.TransInv(T_w_b), T_w_ee)
    for i, joint_name in enumerate(open_chain_joints):
        joint_info = client.getJointInfo(robot, joint_id[joint_name])
        link_name = joint_info[12].decode("utf-8")
        joint_type = joint_info[2]
        joint_axis = joint_info[13]
        screw_at_joint = np.zeros(6)
        link_state = client.getLinkState(robot, link_id[link_name], 1, 1)
        T_w_j = liegroup.RpToTrans(util.quat_to_rot(np.array(link_state[5])),
                                   np.array(link_state[4]))
        T_ee_j = np.dot(liegroup.TransInv(T_w_ee), T_w_j)
        Adj_ee_j = liegroup.Adjoint(T_ee_j)
        if joint_type == client.JOINT_REVOLUTE:
            screw_at_joint[0:3] = np.array(joint_axis)
        elif joint_type == client.JOINT_PRISMATIC:
            screw_at_joint[3:6] = np.array(joint_axis)
        else:
            raise ValueError
        joint_screws_in_ee[:, i] = np.dot(Adj_ee_j, screw_at_joint)

    return joint_screws_in_ee, T_b_ee


def get_link_iso(robot, link_idx, client):
    info = client.getLinkState(robot, link_idx, 1, 1)
    pos = np.array(info[0])
    rot = util.quat_to_rot(np.array(info[1]))

    return liegroup.RpToTrans(rot, pos)


def get_link_vel(robot, link_idx, client):
    info = client.getLinkState(robot, link_idx, 1, 1)
    ret = np.zeros(6)
    ret[3:6] = np.array(info[6])
    ret[0:3] = np.array(info[7])

    return ret


def set_link_damping(robot, link_id, lin_damping, ang_damping, client):
    for i in link_id.values():
        client.changeDynamics(robot,
                          i,
                          linearDamping=lin_damping,
                          angularDamping=ang_damping)


def set_joint_friction(robot, joint_id, max_force=0, client = None):
    client.setJointMotorControlArray(robot,
                                 joint_id.values(),
                                 client.VELOCITY_CONTROL,
                                 forces=[max_force] * len(joint_id))


def draw_link_frame(robot, link_idx, linewidth=5.0, text=None, client = None):
    # This only works when the link has an visual element defined in the urdf file
    if text is not None:
        client.addUserDebugText(text, [0, 0, 0.1],
                            textColorRGB=[1, 0, 0],
                            textSize=1.5,
                            parentObjectUniqueId=robot,
                            parentLinkIndex=link_idx)

    client.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0],
                        linewidth,
                        parentObjectUniqueId=robot,
                        parentLinkIndex=link_idx)

    client.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0],
                        linewidth,
                        parentObjectUniqueId=robot,
                        parentLinkIndex=link_idx)

    client.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1],
                        linewidth,
                        parentObjectUniqueId=robot,
                        parentLinkIndex=link_idx)


def set_motor_impedance(robot, joint_id, command, kp, kd, client):
    trq_applied = OrderedDict()
    for (joint_name, pos_des), (_, vel_des), (_, trq_des) in zip(
            command['joint_positions_cmd_'].items(),
            command['joint_velocities_cmd_'].items(),
            command['joint_torques_cmd_'].items()):
        joint_state = client.getJointState(robot, joint_id[joint_name])
        joint_pos, joint_vel = joint_state[0], joint_state[1]
        trq_applied[joint_id[joint_name]] = trq_des + kp[joint_name] * (
            pos_des - joint_pos) + kd[joint_name] * (vel_des - joint_vel)

    client.setJointMotorControlArray(robot,
                                 trq_applied.keys(),
                                 controlMode=client.TORQUE_CONTROL,
                                 forces=list(trq_applied.values()))


def set_motor_trq(robot, joint_id, trq_cmd, client):
    trq_applied = OrderedDict()
    for joint_name, trq_des in trq_cmd.items():
        trq_applied[joint_id[joint_name]] = trq_des

    client.setJointMotorControlArray(robot,
                                 trq_applied.keys(),
                                 controlMode=client.TORQUE_CONTROL,
                                 forces=list(trq_applied.values()))


def set_motor_pos(robot, joint_id, pos_cmd, client):
    pos_applied = OrderedDict()
    for joint_name, pos_des in pos_cmd.items():
        pos_applied[joint_id[joint_name]] = pos_des

    client.setJointMotorControlArray(robot,
                                 pos_applied.keys(),
                                 controlMode=client.POSITION_CONTROL,
                                 targetPositions=list(pos_applied.values()))


def set_motor_pos_vel(robot, joint_id, pos_cmd, vel_cmd, client):
    pos_applied = OrderedDict()
    vel_applied = OrderedDict()
    for (joint_name, pos_des), (_, vel_des) in zip(pos_cmd.items(),
                                                   vel_cmd.items()):
        pos_applied[joint_id[joint_name]] = pos_des
        vel_applied[joint_id[joint_name]] = vel_des

    client.setJointMotorControlArray(robot,
                                 pos_applied.keys(),
                                 controlMode=client.POSITION_CONTROL,
                                 targetPositions=list(pos_applied.values()),
                                 targetVelocities=list(vel_applied.values()))


def get_sensor_data(robot, joint_id, link_id, pos_basejoint_to_basecom,
                    rot_basejoint_to_basecom, client):
    """
    Parameters
    ----------
    joint_id (dict):
        Joint ID Dict
    link_id (dict):
        Link ID Dict
    pos_basejoint_to_basecom (np.ndarray):
        3d vector from base joint frame to base com frame
    rot_basejoint_to_basecom (np.ndarray):
        SO(3) from base joint frame to base com frame
    b_fixed_Base (bool);
        Whether the robot is floating or fixed
    Returns
    -------
    sensor_data (dict):
        base_com_pos (np.array):
            base com pos in world
        base_com_quat (np.array):
            base com quat in world [x,y,z,w]
        base_com_lin_vel (np.array):
            base com lin vel in world
        base_com_ang_vel (np.array):
            base com ang vel in world
        base_joint_pos (np.array):
            base pos in world
        base_joint_quat (np.array):
            base quat in world
        base_joint_lin_vel (np.array):
            base lin vel in world
        base_joint_ang_vel (np.array):
            base ang vel in world
        joint_pos (dict):
            Joint pos
        joint_vel (dict):
            Joint vel
        b_rf_contact (bool):
            Right Foot Contact Switch
        b_lf_contact (bool):
            Left Foot Contact Switch
    """
    sensor_data = OrderedDict()

    # Handle Base Frame Quantities
    base_com_pos, base_com_quat = client.getBasePositionAndOrientation(robot)
    sensor_data['base_com_pos'] = np.asarray(base_com_pos)
    sensor_data['base_com_quat'] = np.asarray(base_com_quat)

    base_com_lin_vel, base_com_ang_vel = client.getBaseVelocity(robot)
    sensor_data['base_com_lin_vel'] = np.asarray(base_com_lin_vel)
    sensor_data['base_com_ang_vel'] = np.asarray(base_com_ang_vel)

    rot_world_com = util.quat_to_rot(np.copy(sensor_data['base_com_quat']))
    rot_world_joint = np.dot(rot_world_com,
                             rot_basejoint_to_basecom.transpose())
    sensor_data['base_joint_pos'] = sensor_data['base_com_pos'] - np.dot(
        rot_world_joint, pos_basejoint_to_basecom)
    sensor_data['base_joint_quat'] = util.rot_to_quat(rot_world_joint)
    trans_joint_com = liegroup.RpToTrans(rot_basejoint_to_basecom,
                                         pos_basejoint_to_basecom)
    adT_joint_com = liegroup.Adjoint(trans_joint_com)
    twist_com_in_world = np.zeros(6)
    twist_com_in_world[0:3] = np.copy(sensor_data['base_com_ang_vel'])
    twist_com_in_world[3:6] = np.copy(sensor_data['base_com_lin_vel'])
    augrot_com_world = np.zeros((6, 6))
    augrot_com_world[0:3, 0:3] = rot_world_com.transpose()
    augrot_com_world[3:6, 3:6] = rot_world_com.transpose()
    twist_com_in_com = np.dot(augrot_com_world, twist_com_in_world)
    twist_joint_in_joint = np.dot(adT_joint_com, twist_com_in_com)
    rot_world_joint = np.dot(rot_world_com,
                             rot_basejoint_to_basecom.transpose())
    augrot_world_joint = np.zeros((6, 6))
    augrot_world_joint[0:3, 0:3] = rot_world_joint
    augrot_world_joint[3:6, 3:6] = rot_world_joint
    twist_joint_in_world = np.dot(augrot_world_joint, twist_joint_in_joint)
    sensor_data['base_joint_lin_vel'] = np.copy(twist_joint_in_world[3:6])
    sensor_data['base_joint_ang_vel'] = np.copy(twist_joint_in_world[0:3])

    # Joint Quantities
    sensor_data['joint_pos'] = OrderedDict()
    sensor_data['joint_vel'] = OrderedDict()
    for k, v in joint_id.items():
        js = client.getJointState(robot, v)
        sensor_data['joint_pos'][k] = js[0]
        sensor_data['joint_vel'][k] = js[1]

    return sensor_data


def simulate_dVel_data(robot, link_id, previous_link_velocity, client):

    # calculate imu acceleration in world frame by numerical differentiation
    torso_dvel = (get_link_vel(robot, link_id['torso_imu'], client)[3:6] - previous_link_velocity)

    return torso_dvel


def simulate_contact_sensor(force_sensor_measurement):
    """
    Convert PyBullet's sensed force measurements to estimated voltage measurements to
    simulate robot readings.
    :param force_sensor_measurement: Sensed local z-forces from PyBullet
    :return: voltage measurment
    """

    voltage_bias = 25000
    voltage_to_force_map = -10000

    return (force_sensor_measurement - voltage_bias) / voltage_to_force_map

def get_camera_image_from_link(robot, link, pic_width, pic_height, fov,
                               nearval, farval, client):
    aspect = pic_width / pic_height
    projection_matrix = client.computeProjectionMatrixFOV(fov, aspect, nearval,
                                                      farval)
    link_info = client.getLinkState(robot, link, 1, 1)  #Get head link info
    link_pos = link_info[0]  #Get link com pos wrt world
    link_ori = link_info[1]  #Get link com ori wrt world
    rot = client.getMatrixFromQuaternion(link_ori)
    rot = np.array(rot).reshape(3, 3)

    global_camera_x_unit = np.array([1, 0, 0])
    global_camera_z_unit = np.array([0, 0, 1])

    camera_eye_pos = link_pos + np.dot(rot, 0.1 * global_camera_x_unit)
    camera_target_pos = link_pos + np.dot(rot, 1.0 * global_camera_x_unit)
    camera_up_vector = np.dot(rot, global_camera_z_unit)
    view_matrix = client.computeViewMatrix(camera_eye_pos, camera_target_pos,
                                       camera_up_vector)  #SE3_camera_to_world
    width, height, rgb_img, depth_img, seg_img = client.getCameraImage(
        pic_width,  #image width
        pic_height,  #image height
        view_matrix,
        projection_matrix)
    return width, height, rgb_img, depth_img, seg_img, view_matrix, projection_matrix, camera_eye_pos


def add_sensor_noise(measurement, noise):
    return measurement + noise


def make_video(video_dir, delete_jpgs=True):
    images = []
    for file in tqdm(sorted(os.listdir(video_dir)),
                     desc='converting jpgs to gif'):
        filename = video_dir + '/' + file
        im = cv2.imread(filename)
        im = im[:, :, [2, 1, 0]]  # << BGR to RGB
        images.append(im)
        if delete_jpgs:
            os.remove(filename)
    imageio.mimsave(video_dir + '/video.gif', images[:-1], duration=0.01)


def get_camera_image(cam_target_pos, cam_dist, cam_yaw, cam_pitch, cam_roll,
                     fov, render_width, render_height, nearval, farval, client):
    view_matrix = client.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=cam_target_pos,
        distance=cam_dist,
        yaw=cam_yaw,
        pitch=cam_pitch,
        roll=cam_roll,
        upAxisIndex=2)
    proj_matrix = client.computeProjectionMatrixFOV(fov=fov,
                                                aspect=float(render_width) /
                                                float(render_height),
                                                nearVal=nearval,
                                                farVal=farval)
    (_, _, px, _, _) = client.getCameraImage(width=render_width,
                                         height=render_height,
                                         renderer=client.ER_BULLET_HARDWARE_OPENGL,
                                         viewMatrix=view_matrix,
                                         projectionMatrix=proj_matrix)
    rgb_array = np.array(px, dtype=np.uint8)
    rgb_array = np.reshape(np.array(px), (render_height, render_width, -1))
    rgb_array = rgb_array[:, :, :3]

    return rgb_array


def is_key_triggered(keys, key, client):
    o = ord(key)
    if o in keys:
        return keys[ord(key)] & client.KEY_WAS_TRIGGERED
    return False


def set_config(robot, joint_id, base_pos, base_quat, joint_pos, client):
    client.resetBasePositionAndOrientation(robot, base_pos, base_quat)
    for k, v in joint_pos.items():
        client.resetJointState(robot, joint_id[k], v, 0.)


def get_point_cloud_data(depth_buffer, view_matrix, projection_matrix, d_hor,
                         d_ver):
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order='F')
    projection_matrix = np.asarray(projection_matrix).reshape([4, 4],
                                                              order='F')
    trans_world_to_pix = np.linalg.inv(
        np.matmul(projection_matrix, view_matrix))
    trans_camera_to_pix = np.linalg.inv(projection_matrix)
    img_height = (depth_buffer.shape)[0]
    img_width = (depth_buffer.shape)[1]

    wf_point_cloud_data = np.empty(
        [np.int(img_height / d_ver),
         np.int(img_width / d_hor), 3])
    cf_point_cloud_data = np.empty(
        [np.int(img_height / d_ver),
         np.int(img_width / d_hor), 3])

    for h in range(0, img_height, d_ver):
        for w in range(0, img_width, d_hor):
            x = (2 * w - img_width) / img_width
            y = (2 * h - img_height) / img_height
            z = 2 * depth_buffer[h, w] - 1
            pix_pos = np.asarray([x, y, z, 1])
            point_in_world = np.matmul(trans_world_to_pix, pix_pos)
            point_in_camera = np.matmul(trans_camera_to_pix, pix_pos)
            wf_point_cloud_data[np.int(h / d_ver),
                                np.int(w / d_hor), :] = (
                                    point_in_world /
                                    point_in_world[3])[:3]  #world frame

            cf_point_cloud_data[np.int(h / d_ver),
                                np.int(w / d_hor), :] = (
                                    point_in_world /
                                    point_in_world[3])[:3]  #camera frame

    return wf_point_cloud_data, cf_point_cloud_data





def get_sensor_data_from_pybullet(robot, DracoLinkIdx, DracoJointIdx, previous_torso_velocity,
                                    link_id_dict, client):

    #follow pinocchio robotsystem urdf reading convention
    joint_pos, joint_vel = np.zeros(27), np.zeros(27)

    imu_frame_quat = np.array(
        client.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[1])
    #LF
    joint_pos[0] = client.getJointState(robot, DracoJointIdx.l_hip_ie)[0]
    joint_pos[1] = client.getJointState(robot, DracoJointIdx.l_hip_aa)[0]
    joint_pos[2] = client.getJointState(robot, DracoJointIdx.l_hip_fe)[0]
    joint_pos[3] = client.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[0]
    joint_pos[4] = client.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[0]
    joint_pos[5] = client.getJointState(robot, DracoJointIdx.l_ankle_fe)[0]
    joint_pos[6] = client.getJointState(robot, DracoJointIdx.l_ankle_ie)[0]
    #LH
    joint_pos[7] = client.getJointState(robot, DracoJointIdx.l_shoulder_fe)[0]
    joint_pos[8] = client.getJointState(robot, DracoJointIdx.l_shoulder_aa)[0]
    joint_pos[9] = client.getJointState(robot, DracoJointIdx.l_shoulder_ie)[0]
    joint_pos[10] = client.getJointState(robot, DracoJointIdx.l_elbow_fe)[0]
    joint_pos[11] = client.getJointState(robot, DracoJointIdx.l_wrist_ps)[0]
    joint_pos[12] = client.getJointState(robot, DracoJointIdx.l_wrist_pitch)[0]
    #neck
    joint_pos[13] = client.getJointState(robot, DracoJointIdx.neck_pitch)[0]
    #RF
    joint_pos[14] = client.getJointState(robot, DracoJointIdx.r_hip_ie)[0]
    joint_pos[15] = client.getJointState(robot, DracoJointIdx.r_hip_aa)[0]
    joint_pos[16] = client.getJointState(robot, DracoJointIdx.r_hip_fe)[0]
    joint_pos[17] = client.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[0]
    joint_pos[18] = client.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[0]
    joint_pos[19] = client.getJointState(robot, DracoJointIdx.r_ankle_fe)[0]
    joint_pos[20] = client.getJointState(robot, DracoJointIdx.r_ankle_ie)[0]
    #RH
    joint_pos[21] = client.getJointState(robot, DracoJointIdx.r_shoulder_fe)[0]
    joint_pos[22] = client.getJointState(robot, DracoJointIdx.r_shoulder_aa)[0]
    joint_pos[23] = client.getJointState(robot, DracoJointIdx.r_shoulder_ie)[0]
    joint_pos[24] = client.getJointState(robot, DracoJointIdx.r_elbow_fe)[0]
    joint_pos[25] = client.getJointState(robot, DracoJointIdx.r_wrist_ps)[0]
    joint_pos[26] = client.getJointState(robot, DracoJointIdx.r_wrist_pitch)[0]

    imu_ang_vel = np.array(
        client.getLinkState(robot, DracoLinkIdx.torso_imu, 1, 1)[7])

    imu_dvel = simulate_dVel_data(robot, link_id_dict,
                                        previous_torso_velocity, client)

    #LF
    joint_vel[0] = client.getJointState(robot, DracoJointIdx.l_hip_ie)[1]
    joint_vel[1] = client.getJointState(robot, DracoJointIdx.l_hip_aa)[1]
    joint_vel[2] = client.getJointState(robot, DracoJointIdx.l_hip_fe)[1]
    joint_vel[3] = client.getJointState(robot, DracoJointIdx.l_knee_fe_jp)[1]
    joint_vel[4] = client.getJointState(robot, DracoJointIdx.l_knee_fe_jd)[1]
    joint_vel[5] = client.getJointState(robot, DracoJointIdx.l_ankle_fe)[1]
    joint_vel[6] = client.getJointState(robot, DracoJointIdx.l_ankle_ie)[1]
    #LH
    joint_vel[7] = client.getJointState(robot, DracoJointIdx.l_shoulder_fe)[1]
    joint_vel[8] = client.getJointState(robot, DracoJointIdx.l_shoulder_aa)[1]
    joint_vel[9] = client.getJointState(robot, DracoJointIdx.l_shoulder_ie)[1]
    joint_vel[10] = client.getJointState(robot, DracoJointIdx.l_elbow_fe)[1]
    joint_vel[11] = client.getJointState(robot, DracoJointIdx.l_wrist_ps)[1]
    joint_vel[12] = client.getJointState(robot, DracoJointIdx.l_wrist_pitch)[1]
    #neck
    joint_vel[13] = client.getJointState(robot, DracoJointIdx.neck_pitch)[1]
    #RF
    joint_vel[14] = client.getJointState(robot, DracoJointIdx.r_hip_ie)[1]
    joint_vel[15] = client.getJointState(robot, DracoJointIdx.r_hip_aa)[1]
    joint_vel[16] = client.getJointState(robot, DracoJointIdx.r_hip_fe)[1]
    joint_vel[17] = client.getJointState(robot, DracoJointIdx.r_knee_fe_jp)[1]
    joint_vel[18] = client.getJointState(robot, DracoJointIdx.r_knee_fe_jd)[1]
    joint_vel[19] = client.getJointState(robot, DracoJointIdx.r_ankle_fe)[1]
    joint_vel[20] = client.getJointState(robot, DracoJointIdx.r_ankle_ie)[1]
    #RH
    joint_vel[21] = client.getJointState(robot, DracoJointIdx.r_shoulder_fe)[1]
    joint_vel[22] = client.getJointState(robot, DracoJointIdx.r_shoulder_aa)[1]
    joint_vel[23] = client.getJointState(robot, DracoJointIdx.r_shoulder_ie)[1]
    joint_vel[24] = client.getJointState(robot, DracoJointIdx.r_elbow_fe)[1]
    joint_vel[25] = client.getJointState(robot, DracoJointIdx.r_wrist_ps)[1]
    joint_vel[26] = client.getJointState(robot, DracoJointIdx.r_wrist_pitch)[1]

    # normal force measured on each foot
    _l_normal_force = 0
    contacts = client.getContactPoints(bodyA=robot, linkIndexA=DracoLinkIdx.l_ankle_ie_link)
    for contact in contacts:
        # add z-component on all points of contact
        _l_normal_force += contact[9]

    _r_normal_force = 0
    contacts = client.getContactPoints(bodyA=robot, linkIndexA=DracoLinkIdx.r_ankle_ie_link)
    for contact in contacts:
        # add z-component on all points of contact
        _r_normal_force += contact[9]
    
    b_lf_contact = True if client.getLinkState(robot, DracoLinkIdx.l_foot_contact,  #C: change the contact setting from distance to force
                                           1, 1)[0][2] <= 0.005 else False
    b_rf_contact = True if client.getLinkState(robot, DracoLinkIdx.r_foot_contact,
                                           1, 1)[0][2] <= 0.005 else False
    """
    b_lf_contact = True if _l_normal_force > 0 else False
    b_rf_contact = True if _r_normal_force > 0 else False
    """

    return imu_frame_quat, imu_ang_vel, imu_dvel, joint_pos, joint_vel, b_lf_contact, \
        b_rf_contact, _l_normal_force, _r_normal_force


def     apply_control_input_to_pybullet(robot, command, DracoJointIdx, client):
    mode = client.TORQUE_CONTROL

    #LF
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_ie,
                             controlMode=mode,
                             force=command[0])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_aa,
                             controlMode=mode,
                             force=command[1])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_hip_fe,
                             controlMode=mode,
                             force=command[2])
    # client.setJointMotorControl2(robot, DracoJointIdx.l_knee_fe_jp, controlMode=mode, force=command[3])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_knee_fe_jd,
                             controlMode=mode,
                             force=command[4])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_ankle_fe,
                             controlMode=mode,
                             force=command[5])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_ankle_ie,
                             controlMode=mode,
                             force=command[6])

    #LH
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_fe,
                             controlMode=mode,
                             force=command[7])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_aa,
                             controlMode=mode,
                             force=command[8])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_shoulder_ie,
                             controlMode=mode,
                             force=command[9])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_elbow_fe,
                             controlMode=mode,
                             force=command[10])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_wrist_ps,
                             controlMode=mode,
                             force=command[11])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.l_wrist_pitch,
                             controlMode=mode,
                             force=command[12])

    #neck
    client.setJointMotorControl2(robot,
                             DracoJointIdx.neck_pitch,
                             controlMode=mode,
                             force=command[13])
    #RF
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_ie,
                             controlMode=mode,
                             force=command[14])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_aa,
                             controlMode=mode,
                             force=command[15])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_hip_fe,
                             controlMode=mode,
                             force=command[16])
    # client.setJointMotorControl2(robot, DracoJointIdx.r_knee_fe_jd, controlMode=mode, force=command[17])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_knee_fe_jd,
                             controlMode=mode,
                             force=command[18])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_ankle_fe,
                             controlMode=mode,
                             force=command[19])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_ankle_ie,
                             controlMode=mode,
                             force=command[20])

    #RH
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_fe,
                             controlMode=mode,
                             force=command[21])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_aa,
                             controlMode=mode,
                             force=command[22])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_shoulder_ie,
                             controlMode=mode,
                             force=command[23])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_elbow_fe,
                             controlMode=mode,
                             force=command[24])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_wrist_ps,
                             controlMode=mode,
                             force=command[25])
    client.setJointMotorControl2(robot,
                             DracoJointIdx.r_wrist_pitch,
                             controlMode=mode,
                             force=command[26])


