"""A utility class to collect data during demonstration and saves it to an HDF5 file.
"""
import collections
import numpy as np
import h5py
import time
from datetime import datetime


class DemoCollector:
    def __init__(self, demonstrator, task):
        self.demonstrator = demonstrator
        self.task = task
        # record the saved data real-time in a buffer in memory for performance
        self.vr_ready_prev = False
        self.clear_buffer()

    def save_data(self, msg, rgb_img, stereo_img):
        if msg.vr_ready:
            # control pc data
            self.data_buffer['obs/joint_pos'].append(list(msg.joint_positions))
            self.data_buffer['obs/joint_vel'].append(
                list(msg.joint_velocities))

            # hand and feet poses
            self.data_buffer['obs/act_global_lh_pos'].append(
                list(msg.act_global_lh_pos))
            self.data_buffer['obs/act_global_rh_pos'].append(
                list(msg.act_global_rh_pos))
            self.data_buffer['obs/act_global_lf_pos'].append(
                list(msg.act_global_lf_pos))
            self.data_buffer['obs/act_global_rf_pos'].append(
                list(msg.act_global_rf_pos))
            self.data_buffer['obs/act_global_lh_ori'].append(
                list(msg.act_global_lh_ori))
            self.data_buffer['obs/act_global_rh_ori'].append(
                list(msg.act_global_rh_ori))
            self.data_buffer['obs/act_global_lf_ori'].append(
                list(msg.act_global_lf_ori))
            self.data_buffer['obs/act_global_rf_ori'].append(
                list(msg.act_global_rf_ori))

            self.data_buffer['obs/des_global_lh_pos'].append(
                list(msg.des_global_lh_pos))
            self.data_buffer['obs/des_global_rh_pos'].append(
                list(msg.des_global_rh_pos))
            self.data_buffer['obs/des_global_lf_pos'].append(
                list(msg.des_global_lf_pos))
            self.data_buffer['obs/des_global_rf_pos'].append(
                list(msg.des_global_rf_pos))
            self.data_buffer['obs/des_global_lh_ori'].append(
                list(msg.des_global_lh_ori))
            self.data_buffer['obs/des_global_rh_ori'].append(
                list(msg.des_global_rh_ori))
            self.data_buffer['obs/des_global_lf_ori'].append(
                list(msg.des_global_lf_ori))
            self.data_buffer['obs/des_global_rf_ori'].append(
                list(msg.des_global_rf_ori))

            self.data_buffer['obs/state'].append(msg.state)
            self.data_buffer['action/l_gripper'].append(msg.l_gripper)
            self.data_buffer['action/r_gripper'].append(msg.r_gripper)
            self.data_buffer['action/local_lh_pos'].append(
                list(msg.action_local_lh_pos))
            self.data_buffer['action/local_lh_ori'].append(
                list(msg.action_local_lh_ori))
            self.data_buffer['action/local_rh_pos'].append(
                list(msg.action_local_rh_pos))
            self.data_buffer['action/local_rh_ori'].append(
                list(msg.action_local_rh_ori))
            self.data_buffer['est_base_joint_pos'].append(
                list(msg.est_base_joint_pos))
            self.data_buffer['est_base_joint_ori'].append(
                list(msg.est_base_joint_ori))
            self.data_buffer['kf_base_joint_pos'].append(
                list(msg.kf_base_joint_pos))
            self.data_buffer['kf_base_joint_ori'].append(
                list(msg.kf_base_joint_ori))
            self.data_buffer['timestamp'].append(msg.timestamp)

            # camera data
            self.data_buffer['obs/rgb'].append(np.frombuffer(rgb_img,
                                                             dtype=np.uint8).reshape(200, 400, 3))
            self.data_buffer['obs/stereo'].append(np.frombuffer(
                stereo_img, dtype=np.uint8).reshape(200, 800, 1))
        elif self.vr_ready_prev:
            # if we go from controlling the robot to not controlling the robot, save it
            print("Saving data...")
            timestamp = time.time()
            date_time = datetime.fromtimestamp(timestamp)
            str_date_time = date_time.strftime("%d-%m-%Y--%H:%M:%S")
            # save data_buffer to hdf5 file
            with h5py.File(f"{self.demonstrator}_{self.task}_{str_date_time}.hdf5", 'w') as f:
                for key, value in self.data_buffer.items():
                    f.create_dataset(key, data=np.array(value))
            self.clear_buffer()
            print("Done!")

        self.vr_ready_prev = msg.vr_ready

    def clear_buffer(self):
        self.data_buffer = {'obs/joint_pos': collections.deque(),
                            'obs/joint_vel': collections.deque(),
                            'obs/act_global_lh_pos': collections.deque(),
                            'obs/act_global_rh_pos': collections.deque(),
                            'obs/act_global_lf_pos': collections.deque(),
                            'obs/act_global_rf_pos':  collections.deque(),
                            'obs/act_global_lh_ori': collections.deque(),
                            'obs/act_global_rh_ori': collections.deque(),
                            'obs/act_global_lf_ori': collections.deque(),
                            'obs/act_global_rf_ori': collections.deque(),

                            'obs/des_global_lh_pos': collections.deque(),
                            'obs/des_global_rh_pos': collections.deque(),
                            'obs/des_global_lf_pos': collections.deque(),
                            'obs/des_global_rf_pos':  collections.deque(),
                            'obs/des_global_lh_ori': collections.deque(),
                            'obs/des_global_rh_ori': collections.deque(),
                            'obs/des_global_lf_ori': collections.deque(),
                            'obs/des_global_rf_ori': collections.deque(),

                            'obs/state': collections.deque(),
                            'obs/rgb': collections.deque(),
                            'obs/stereo': collections.deque(),
                            'action/l_gripper': collections.deque(),
                            'action/r_gripper': collections.deque(),
                            'action/local_lh_pos': collections.deque(),
                            'action/local_lh_ori': collections.deque(),
                            'action/local_rh_pos': collections.deque(),
                            'action/local_rh_ori': collections.deque(),
                            'est_base_joint_pos': collections.deque(),
                            'est_base_joint_ori': collections.deque(),
                            'kf_base_joint_pos': collections.deque(),
                            'kf_base_joint_ori': collections.deque(),
                            'timestamp': collections.deque(),
                            }
