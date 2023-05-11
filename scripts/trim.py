"""This is a script to generate a video with all demonstration sequences"""
import numpy as np
import h5py
import numpy as np
import torch
import torchvision
import numpy as np
import math
import argparse
import cv2

DATA_KEYS = []

DATA_KEYS += ['global_base_ori', 'global_base_pos',
    'collection_timestamp', 'rgb_time', 'rpc_time', 'rpc_timestamp', 'stereo_time', 'vr_timestamp']

DATA_KEYS += ['action/{}'.format(action_key) for action_key in [
    'l_gripper', 'local_lh_ori', 'local_lh_pos', 'local_rh_ori', 'local_rh_pos', 'r_gripper']]

DATA_KEYS += ['obs/{}'.format(obs_key) for obs_key in [
'act_global_lf_ori', 'act_global_lf_pos', 'act_global_lh_ori', 'act_global_lh_pos', 'act_global_rf_ori', 'act_global_rf_pos', 'act_global_rh_ori', 'act_global_rh_pos', 'des_global_lf_ori', 'des_global_lf_pos', 'des_global_lh_ori', 'des_global_lh_pos', 'des_global_rf_ori', 'des_global_rf_pos', 'des_global_rh_ori', 'des_global_rh_pos', 
'joint_pos', 'joint_vel', 'rgb', 'state', 'stereo'
    ]]

DATA_KEYS += ['est_base_joint_ori', 'est_base_joint_pos', 'kf_base_joint_ori', 'kf_base_joint_pos']


def main(path, dimensions=None):
    demo_file_name = path
    demo_file = h5py.File(demo_file_name, "r")
    print(demo_file.keys())

    images = demo_file[f"obs/rgb"][()]

    # new_demo_file_name = "new_" + path 
    # new_demo_file = h5py.File(new_demo_file_name, "w")
    
    start_idx = 0
    
    for idx in range(images.shape[0]):
        cv2.imshow("test", images[idx])
        cv2.waitKey(1)
        if demo_file["action/r_gripper"][idx]:
            keyboard_input = input("Press enter to continue, q to quit: ")

            if keyboard_input == "":
                pass
            if keyboard_input == "s":
                start_idx = idx
            if keyboard_input == "q":
                end_idx = idx
                break
        if idx == images.shape[0] - 1:
            end_idx = idx
        print(idx, demo_file["action/r_gripper"][idx], demo_file["obs/act_global_rh_pos"][idx])
    
    print(demo_file["obs"].keys())
    print(demo_file["action"].keys())
    
    # for key in DATA_KEYS:
    #     value = np.array(demo_file[key][start_idx:end_idx])
    #     # if key == 'action/r_gripper':
    #     #     value[-1] = 0
    #     new_demo_file.create_dataset(key, data=value)

    # new_demo_file.close()        
    demo_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=str)
    args = parser.parse_args()

    main(args.path)
