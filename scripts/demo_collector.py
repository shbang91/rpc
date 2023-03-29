"""A utility class to collect data during demonstration and saves it to an HDF5 file.
"""
import collections
import numpy as np
import h5py


class DemoCollector:
    def __init__(self, filename):
        self.filename = filename
        # record the saved data real-time in a buffer in memory for performance
        self.data_buffer = {'obs/joint_pos': collections.deque(),
                            'obs/joint_vel': collections.deque(),
                            'obs/local_lh_pos': collections.deque(),
                            'obs/local_rh_pos': collections.deque(),
                            'obs/local_lf_pos': collections.deque(),
                            'obs/local_rf_pos':  collections.deque(),
                            'obs/local_lh_ori': collections.deque(),
                            'obs/local_rh_ori': collections.deque(),
                            'obs/local_lf_ori': collections.deque(),
                            'obs/local_rf_ori': collections.deque(),
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
        self.vr_ready_prev = False

    def save_data(self, msg, rgb_img, stereo_img):
        if msg.vr_ready:
            # control pc data
            self.data_buffer['obs/joint_pos'].append(list(msg.joint_positions))
            self.data_buffer['obs/joint_vel'].append(
                list(msg.joint_velocities))
            self.data_buffer['obs/local_lh_pos'].append(list(msg.local_lh_pos))
            self.data_buffer['obs/local_rh_pos'].append(list(msg.local_rh_pos))
            self.data_buffer['obs/local_lf_pos'].append(list(msg.local_lf_pos))
            self.data_buffer['obs/local_rf_pos'].append(list(msg.local_rf_pos))
            self.data_buffer['obs/local_lh_ori'].append(list(msg.local_lh_ori))
            self.data_buffer['obs/local_rh_ori'].append(list(msg.local_rh_ori))
            self.data_buffer['obs/local_lf_ori'].append(list(msg.local_lf_ori))
            self.data_buffer['obs/local_rf_ori'].append(list(msg.local_rf_ori))
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
            # save data_buffer to hdf5 file
            with h5py.File(self.filename, 'w') as f:
                for key, value in self.data_buffer.items():
                    f.create_dataset(key, data=np.array(value))
            print("Done!")

        self.vr_ready_prev = msg.vr_ready
