"""A utility class to collect data during demonstration and saves it to an HDF5 file.
"""
import collections
import numpy as np
import h5py
import time
from datetime import datetime

def convert_protobuf_and_image(msg, rgb, stereo):
    output = {}
    output['obs/joint_pos'] = np.array(msg.joint_positions)
    output['obs/joint_vel'] = np.array(msg.joint_velocities)
    # hand and feet poses
    output['obs/act_global_lh_pos'] = np.array(msg.act_global_lh_pos)
    output['obs/act_global_rh_pos'] = np.array(msg.act_global_rh_pos)
    output['obs/act_global_lf_pos'] = np.array(msg.act_global_lf_pos)
    output['obs/act_global_rf_pos'] = np.array(msg.act_global_rf_pos)
    output['obs/act_global_lh_ori'] = np.array(msg.act_global_lh_ori)
    output['obs/act_global_rh_ori'] = np.array(msg.act_global_rh_ori)
    output['obs/act_global_lf_ori'] = np.array(msg.act_global_lf_ori)
    output['obs/act_global_rf_ori'] = np.array(msg.act_global_rf_ori)
    output['obs/des_global_lh_pos'] = np.array(msg.des_global_lh_pos)
    output['obs/des_global_rh_pos'] = np.array(msg.des_global_rh_pos)
    output['obs/des_global_lf_pos'] = np.array(msg.des_global_lf_pos)
    output['obs/des_global_rf_pos'] = np.array(msg.des_global_rf_pos)
    output['obs/des_global_lh_ori'] = np.array(msg.des_global_lh_ori)
    output['obs/des_global_rh_ori'] = np.array(msg.des_global_rh_ori)
    output['obs/des_global_lf_ori'] = np.array(msg.des_global_lf_ori)
    output['obs/des_global_rf_ori'] = np.array(msg.des_global_rf_ori)
    # other metadata
    output['obs/state'] = msg.state
    output['est_base_joint_pos'] = np.array(msg.est_base_joint_pos)
    output['est_base_joint_ori'] = np.array(msg.est_base_joint_ori)
    output['kf_base_joint_pos'] = np.array(msg.kf_base_joint_pos)
    output['kf_base_joint_ori'] = np.array(msg.kf_base_joint_ori)
    output['global_base_pos'] = np.array(msg.global_base_pos)
    output['global_base_ori'] = np.array(msg.global_base_ori)
    output['timestamp'] = msg.timestamp
    # actions
    output['action/l_gripper'] = msg.l_gripper
    output['action/r_gripper'] = msg.r_gripper
    output['action/local_lh_pos'] = np.array(msg.action_local_lh_pos)
    output['action/local_lh_ori'] = np.array(msg.action_local_lh_ori)
    output['action/local_rh_pos'] = np.array(msg.action_local_rh_pos)
    output['action/local_rh_ori'] = np.array(msg.action_local_rh_ori)
    # images
    output['obs/rgb'] = rgb
    output['obs/stereo'] = stereo
    return output

class DemoCollector:
    def __init__(self, demonstrator, task):
        self.demonstrator = demonstrator
        self.task = task
        # record the saved data real-time in a buffer in memory for performance
        self.vr_ready_prev = False
        self.clear_buffer()

    def save_data(self, msg, rgb_img, stereo_img):
        if msg.vr_ready:
            converted_data = convert_protobuf_and_image(msg, rgb_img, stereo_img)
            for key in converted_data.keys():
                self.data_buffer[key].append(converted_data)
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
                            'global_base_pos': collections.deque(),
                            'global_base_ori': collections.deque(),
                            'timestamp': collections.deque(),
                            }
