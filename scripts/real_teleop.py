import os
import sys
import argparse
import numpy as np
import io
from pynput import keyboard

cwd = os.getcwd()
sys.path.append(cwd)

from util.python_utils.device.t265 import T265
import util.python_utils.util as util

import copy
import time
import cv2
import csv
from scipy.spatial.transform import Rotation as R
# from util.python_utils.comm import ZMQServer
from scripts.draco_manipulation_comm import DracoZMQServer
import util.python_utils.demo as demo

TARGET_IP = "*"
PUB_PORT = 5555
SUB_PORT = 6000

FPS = 20

# Define colors for each axis
AXIS_COLORS = ['r', 'g', 'b']  # Red for x, Green for y, Blue for z


## Define the thread receiving keyboard for debugging
class Keyboard():

    def __init__(self):

        self.single_click_and_hold = False

        self._reset_state = 0
        self._enabled = False

        self._flag_init = False
        self._t_last_click = -1
        self._t_click = -1

        self._succeed = False
        self._grasp = False

        # launch a new listener thread to listen to keyboard
        self.thread = keyboard.Listener(on_press=self._on_press,
                                        on_release=self._on_release)
        self.thread.start()

    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset grasp
        self.single_click_and_hold = False

        self._flag_init = False
        self._t_last_click = -1
        self._t_click = -1

    def _on_press(self, key):

        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if key_char == 'e':
            self._t_last_click = -1
            self._t_click = time.time()
            elapsed_time = self._t_click - self._t_last_click
            self._t_last_click = self._t_click
            self.single_click_and_hold = True
        elif key_char == 's':
            self._succeed = True
            print('Recording successful')

    def _on_release(self, key):

        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if key_char == 'e':
            self.single_click_and_hold = False

        elif key == keyboard.Key.esc or key_char == 'q':
            self._reset_state = 1
            self._enabled = False
            self._reset_internal_state()

        elif key_char == 'r':
            self._reset_state = 1
            self._enabled = True

        elif key_char == 'c':
            self._reset_state = 1
            self._grasp = True

        elif key_char == 'o':
            self._reset_state = 1
            self._grasp = False

    @property
    def click(self):
        """
        Maps internal states into gripper commands.
        Returns:
            float: Whether we're using single click and hold or not
        """
        if self.single_click_and_hold:
            return 1.0
        return 0

    @property
    def enable(self):
        return self._enabled

    @property
    def succeed(self):
        return self._succeed

    @property
    def grasp(self):
        return self._grasp


def record(path, ros_socket=None):

    t265 = T265()
    keyboard = Keyboard()
    mat_trans = np.eye(4)
    mat_trans[:3, :3] = util.quat_to_rot([0.5, -0.5, -0.5, 0.5])  #T265
    trk_init = True

    done = False
    while not keyboard.enable:
        pass

    t265.start()
    read_time = 0

    # state = {key: [] for key in ['time', 'left_img', 'right_img', 'trk_pos', 'trk_rot', 'robot_pos', 'robot_rot', 'robot_grasp']}

    dir_name = "{}".format(int(time.time()))
    dir_path = os.path.join(path, dir_name)
    os.makedirs(dir_path, exist_ok=True)

    if t265.b_img_stream:
        left_img_path = os.path.join(dir_path, 'left_img')
        right_img_path = os.path.join(dir_path, 'right_img')
        os.makedirs(left_img_path, exist_ok=True)
        os.makedirs(right_img_path, exist_ok=True)

    low_dim_file = open(os.path.join(dir_path, 'low_dim.csv'), 'w')
    low_dim_file.write("time, \
                       pos_x, pos_y, pos_z, \
                       quat_x, quat_y, quat_z, quat_w, \
                       \n")
    # low_dim_file.write("time, \
    # pos_x, pos_y, pos_z, \
    # quat_x, quat_y, quat_z, quat_w, \
    # robot_pos_x, robot_pos_y, robot_pos_z, \
    # robot_quat_x, robot_quat_y, robot_quat_z, robot_quat_w, \
    # robot_grasp\
    # \n")
    # 0 3 7 10 14 15

    pos = np.zeros(3)
    quat = np.array([0, 0, 0, 1])

    while not done:

        if t265.time > read_time + 0.1:
            trk_pos = t265.pos
            trk_rot = t265.rot

            if t265.b_img_stream:
                left_img = np.copy(t265.left)
                right_img = np.copy(t265.right)

            read_time += 0.1
            # print("[{:.3f}], pos: {}, quat: {}".format(t265.time, pos.round(3),
            # quat.round(3)))

            mat_se3 = np.eye(4)
            mat_se3[:3, :3] = util.quat_to_rot(trk_rot)
            mat_se3[:3, 3] = trk_pos

            if trk_init:
                mat_se3_base = np.eye(4)
                mat_se3_base = np.linalg.inv(
                    mat_trans @ mat_se3) @ mat_se3_base
                trk_init = False
                print("Tracking initialized... Start now")

            trk_mat_se3 = mat_trans @ mat_se3 @ mat_se3_base
            pos = trk_mat_se3[:3, 3]
            quat = util.rot_to_quat(trk_mat_se3[:3, :3])
            b_grasp = keyboard.grasp
            b_teleop_toggled = keyboard.enable

            ros_socket.update({
                "pos": pos,
                "quat": quat,
                "b_grasp": b_grasp,
                "b_teleop_toggled": b_teleop_toggled
            })

            # state['time'] += [t265.time]
            # state['left_img'] += [[left_img]]
            # state['right_img'] += [[right_img]]
            # state['trk_pos'] += [pos]
            # state['trk_rot'] += [quat]
            # state['robot_pos'] += [robot_pos]
            # state['robot_rot'] += [robot_quat]
            # state['grasp'] += [robot_grasp]

            done = not keyboard.enable

            if not trk_init:

                low_dim_file.write("{}, \
                                    {}, {}, {},\
                                    {}, {}, {}, {},\
                                \n".format(
                    t265.time,
                    pos[0],
                    pos[1],
                    pos[2],
                    quat[0],
                    quat[1],
                    quat[2],
                    quat[3],
                ))

                if t265.b_img_stream:
                    cv2.imwrite(
                        os.path.join(left_img_path,
                                     "{}.png".format(int(t265.time * 100))),
                        left_img,
                    )
                    cv2.imwrite(
                        os.path.join(right_img_path,
                                     "{}.png".format(int(t265.time * 100))),
                        right_img,
                    )

    t265.stop()

    if input("Do you want to keep the data? (y/n): ") == "n":
        os.system("rm -rf {}".format(dir_path))


def visualize(data_path):

    low_dim_file = open(os.path.join(data_path, 'low_dim.csv'), 'r')
    low_dim_reader = csv.reader(low_dim_file)
    left_img_dir = os.path.join(data_path, 'left_img')
    right_img_dir = os.path.join(data_path, 'right_img')

    # first row is the header
    # create a dict with the header names as keys and save values of the lines to corresponding keys
    data = {
        "time": [],
        "trk_pos": [],
        "trk_rot": [],
        "robot_pos": [],
        "robot_rot": [],
    }

    next(low_dim_reader)
    for row in low_dim_reader:
        data['time'] += [float(row[0])]
        data['trk_pos'] += [[float(row[1]), float(row[2]), float(row[3])]]
        data['trk_rot'] += [[
            float(row[4]),
            float(row[5]),
            float(row[6]),
            float(row[7])
        ]]
        data["robot_pos"] += [[float(row[8]), float(row[9]), float(row[10])]]
        data["robot_rot"] += [[
            float(row[11]),
            float(row[12]),
            float(row[13]),
            float(row[14])
        ]]

    for key in data.keys():
        data[key] = np.array(data[key])

    mat_tool_offset = np.eye(4)
    mat_tool_offset[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    mat_tool_offset[:3, 3] = np.zeros(3)

    data['left_img'] = []
    data['right_img'] = []
    left_img = np.zeros((800, 848, 3), dtype="uint8")
    right_img = np.zeros((800, 848, 3), dtype="uint8")
    for time in data['time']:
        left_img_read = cv2.imread(
            os.path.join(left_img_dir, "{}.png".format(int(time * 100))))
        right_img_read = cv2.imread(
            os.path.join(right_img_dir, "{}.png".format(int(time * 100))))
        data['left_img'] += [left_img]
        data['right_img'] += [right_img]
        # print(right_img)
        if left_img_read is not None:
            left_img = left_img_read
        if right_img_read is not None:
            right_img = right_img_read
        # print(left_img.shape, right_img.shape)

    time = np.array(data['time'][:])
    left_img = np.array(data['left_img'][:])
    right_img = np.array(data['right_img'][:])
    trk_pos = np.array(data["trk_pos"][:])
    trk_rot = np.array(data["trk_rot"][:])
    # ipdb.set_trace()

    robot_pos = np.array(data["robot_pos"][:])
    robot_rot = np.array(data["robot_rot"][:])

    cmd_pos = []
    cmd_rot = []
    for i in range(len(robot_pos)):
        robot_mat = np.eye(4)
        robot_mat[:3, :3] = util.quat_to_rot(robot_rot[i])
        robot_mat[:3, 3] = robot_pos[i]
        cam_mat = robot_mat @ np.linalg.inv(mat_tool_offset)
        cmd_pos.append(cam_mat[:3, 3])
        cmd_rot.append(util.rot_to_quat(cam_mat[:3, :3]))
    cmd_pos = np.array(cmd_pos)
    cmd_rot = np.array(cmd_rot)

    initial_mat = np.eye(4)
    initial_mat[:3, :3] = util.quat_to_rot(robot_rot[0])
    initial_mat[:3, 3] = robot_pos[0]

    trk_action = []
    for i in range(trk_pos.shape[0] - 1):
        pose_vec_cur = np.concatenate([trk_pos[i + 1], trk_rot[i + 1]])
        pose_vec_prv = np.concatenate([trk_pos[i], trk_rot[i]])
        action = demo.reconstruct_pose(pose_vec_cur, pose_vec_prv, mode='quat')
        trk_action.append(action)
    trk_action = np.array(trk_action)

    cmd_action = []
    for i in range(cmd_pos.shape[0] - 1):
        pose_vec_cur = np.concatenate([cmd_pos[i + 1], cmd_rot[i + 1]])
        pose_vec_prv = np.concatenate([cmd_pos[i], cmd_rot[i]])
        action = demo.reconstruct_pose(pose_vec_cur, pose_vec_prv, mode='quat')
        cmd_action.append(action)
    cmd_action = np.array(cmd_action)

    estimated_pos = []
    estimated_rot = []
    pose_ee_vector_prv = np.concatenate(
        [mat_tool_offset[:3, 3],
         util.rot_to_quat(mat_tool_offset[:3, :3])])
    for i in range(trk_action.shape[0] - 1):
        delta_pose_vector = trk_action[i]
        pose_ee_vector_cur = demo.reconstruct_pose(delta_pose_vector,
                                                   pose_ee_vector_prv,
                                                   mode="quat")
        cam_pose_mat = np.eye(4)
        cam_pose_mat[:3, :3] = util.quat_to_rot(pose_ee_vector_cur[3:])
        cam_pose_mat[:3, 3] = pose_ee_vector_cur[:3]
        robot_pose_mat = cam_pose_mat @ mat_tool_offset
        estimated_pos.append(robot_pose_mat[:3, 3])
        estimated_rot.append(util.rot_to_quat(robot_pose_mat[:3, :3]))
    estimated_pos = np.array(estimated_pos)
    estimated_rot = np.array(estimated_rot)

    reconstructed_pos = []
    reconstructed_rot = []
    pose_ee_vector_prv = np.concatenate(
        [mat_tool_offset[:3, 3],
         util.rot_to_quat(mat_tool_offset[:3, :3])])
    for i in range(cmd_action.shape[0] - 1):
        delta_pose_vector = cmd_action[i]
        pose_ee_vector_cur = demo.reconstruct_pose(delta_pose_vector,
                                                   pose_ee_vector_prv,
                                                   mode="quat")
        cam_pose_mat = np.eye(4)
        cam_pose_mat[:3, :3] = util.quat_to_rot(pose_ee_vector_cur[3:])
        cam_pose_mat[:3, 3] = pose_ee_vector_cur[:3]
        robot_pose_mat = cam_pose_mat @ mat_tool_offset
        reconstructed_pos.append(robot_pose_mat[:3, 3])
        reconstructed_rot.append(util.rot_to_quat(robot_pose_mat[:3, :3]))
    reconstructed_pos = np.array(reconstructed_pos)
    reconstructed_rot = np.array(reconstructed_rot)

    fig = plt.figure()
    ax1 = fig.add_subplot(221)
    ax2 = fig.add_subplot(222)
    ax3 = fig.add_subplot(223, projection='3d')
    ax4 = fig.add_subplot(224, projection='3d')

    length = time.shape[0]
    img_buffer = []

    def get_img_from_fig(fig, dpi=360):
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=dpi)
        buf.seek(0)
        img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        buf.close()
        img = cv2.imdecode(img_arr, 1)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2gray)
        return img

    for num in range(length):

        # Prepare the 2D plot for visual observation
        pos_data_3 = estimated_pos[:num + 1, :]
        rot_data_3 = estimated_rot[:num + 1, :]

        pos_data_4 = reconstructed_pos[:num + 1, :]
        rot_data_4 = reconstructed_rot[:num + 1, :]

        left_data = left_img[num]
        right_data = right_img[num]

        fig.suptitle('Time: %.2f' % time[num])
        fig.set_size_inches(5, 2.5)
        if num == 0:
            # fig.set_size_inches(15, 7)
            left_image = ax1.imshow(left_img[0])
            ax1.set_title('Left image', fontdict={'fontsize': 8})
            ax1.tick_params(left=False,
                            right=False,
                            labelleft=False,
                            labelbottom=False,
                            bottom=False)

            right_image = ax2.imshow(right_img[0])
            ax2.set_title('Right image', fontdict={'fontsize': 8})
            ax2.tick_params(left=False,
                            right=False,
                            labelleft=False,
                            labelbottom=False,
                            bottom=False)

            line_3, = ax3.plot(pos_data_3[0, 0],
                               pos_data_3[0, 1],
                               pos_data_3[0, 2],
                               color='blue')
            # point_3, = ax3.plot([], [], [], 'go')
            axis_3 = []
            for j in range(3):
                axis_j, = ax3.plot([], [], [], color=AXIS_COLORS[j])
                axis_3.append(axis_j)
            ax3.set_xlim(pos_data_3[0, 0] - 0.6, pos_data_3[0, 0] + 0.6)
            ax3.set_ylim(pos_data_3[0, 1] - 0.6, pos_data_3[0, 1] + 0.6)
            ax3.set_zlim(pos_data_3[0, 2] - 0.6, pos_data_3[0, 2] + 0.6)
            ax3.xaxis.set_tick_params(labelsize=4, pad=0)
            ax3.yaxis.set_tick_params(labelsize=4, pad=0)
            ax3.zaxis.set_tick_params(labelsize=4, pad=0)
            ax3.set_title('Tracker trajectory', fontdict={'fontsize': 8})
            ax3.grid(True)

            line_4, = ax4.plot(pos_data_4[0, 0],
                               pos_data_4[0, 1],
                               pos_data_4[0, 2],
                               color='blue')
            # point_4, = ax4.plot([], [], [], 'go')
            axis_4 = []
            for j in range(3):
                axis_j, = ax4.plot([], [], [], color=AXIS_COLORS[j])
                axis_4.append(axis_j)
            ax4.set_xlim(pos_data_4[0, 0] - 0.6, pos_data_4[0, 0] + 0.6)
            ax4.set_ylim(pos_data_4[0, 1] - 0.6, pos_data_4[0, 1] + 0.6)
            ax4.set_zlim(pos_data_4[0, 2] - 0.6, pos_data_4[0, 2] + 0.6)
            ax4.xaxis.set_tick_params(labelsize=4, pad=0)
            ax4.yaxis.set_tick_params(labelsize=4, pad=0)
            ax4.zaxis.set_tick_params(labelsize=4, pad=0)
            ax4.set_title('Command trajectory', fontdict={'fontsize': 8})
            ax4.grid(True)

        left_image.set_array(left_data)
        right_image.set_array(right_data)

        line_3.set_data(pos_data_3[:, 0], pos_data_3[:, 1])
        line_3.set_3d_properties(pos_data_3[:, 2])
        # Create a basis in the orientation of the quaternion
        rot_3 = R.from_quat(rot_data_3[-1])
        basis_3 = rot_3.apply(np.eye(3))
        # Plot each basis vector
        for j in range(3):
            start_point = pos_data_3[-1]
            end_point = pos_data_3[-1] + 0.1 * basis_3[j]
            axis_3[j].set_data([start_point[0], end_point[0]],
                               [start_point[1], end_point[1]])
            axis_3[j].set_3d_properties([start_point[2], end_point[2]])
        # point_3.set_data(pos_data_3[-1,0], pos_data_3[-1,1])
        # point_3.set_3d_properties(pos_data_3[-1,2])

        line_4.set_data(pos_data_4[:, 0], pos_data_4[:, 1])
        line_4.set_3d_properties(pos_data_4[:, 2])
        # Create a basis in the orientation of the quaternion
        rot_4 = R.from_quat(rot_data_4[-1])
        basis_4 = rot_4.apply(np.eye(3))
        # Plot each basis vector
        for j in range(3):
            start_point = pos_data_4[-1]
            end_point = pos_data_4[-1] + 0.1 * basis_4[j]
            axis_4[j].set_data([start_point[0], end_point[0]],
                               [start_point[1], end_point[1]])
            axis_4[j].set_3d_properties([start_point[2], end_point[2]])
        # point_4.set_data(pos_data_4[-1,0], pos_data_4[-1,1])
        # point_4.set_3d_properties(pos_data_4[-1,2])

        img = get_img_from_fig(fig)
        img_buffer.append(img)

    save_path = os.path.join(data_path, "visualize_trk.mp4")
    video_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                   30, img_buffer[0].shape[1::-1])
    for img in img_buffer:
        video_writer.write(img)
    video_writer.release()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, default='./data/test')
    args = parser.parse_args()
    path = args.path

    server = DracoZMQServer(ip=TARGET_IP,
                            pub_port=PUB_PORT,
                            sub_port=SUB_PORT,
                            verbose=True)
    server.start()

    demo_cnt = 1

    while True:
        print("Recording {}th demo...".format(demo_cnt))
        record(path, ros_socket=server)
        print("Pause, waiting for next demo...")
        demo_cnt += 1

    server.stop()
