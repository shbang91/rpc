"""A script to execute the neural network policy on the real robot
or saves the demonstration data for training. 

This script will listen to 2 zmq queues: one from the control PC and
the other one from the camera C++ script (which has 2 sockets for the 
rgb images and stereo grayscale images). It then syncs the data
and either saves them into an HDF5 file or executes the neural network

Alternatively, the input data can be taken from a replay file. 

The data is saved in the following format:
├── action
│   ├── l_gripper 
│   ├── local_lh_ori 
│   ├── local_lh_pos 
│   ├── local_rh_ori 
│   ├── local_rh_pos 
│   └── r_gripper 
├── est_base_joint_ori 
├── est_base_joint_pos 
├── kf_base_joint_ori 
├── kf_base_joint_pos 
├── obs
│   ├── joint_pos 
│   ├── joint_vel 
│   ├── local_lf_ori 
│   ├── local_lf_pos 
│   ├── local_lh_ori 
│   ├── local_lh_pos 
│   ├── local_rf_ori 
│   ├── local_rf_pos 
│   ├── local_rh_ori 
│   ├── local_rh_pos 
│   ├── rgb 
│   ├── state 
│   └── stereo 
└── timestamp 


"""
import numpy as np
import argparse
import zmq
import time

from scripts.demo_collector import DemoCollector
from scripts.input_data_manager import InputDataManager, SocketInputDataManager


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save_data", type=bool, default=True,
                        help="True if we are collect data for training. False if we are executing the policy.")
    parser.add_argument("--control_ip", type=str,
                        default="tcp://192.168.1.184:5557")
    parser.add_argument("--rgb_camera_ip", type=str,
                        default="tcp://localhost:5557")
    parser.add_argument("--stereo_camera_ip", type=str,
                        default="tcp://localhost:5558")
    parser.add_argument("--demonstrator", type=str, default="steve")
    parser.add_argument("--task", type=str, default="screwdriver")
    args = parser.parse_args()
    save_data = args.save_data

    # Socket initialize
    context = zmq.Context()
    input_manager = SocketInputDataManager(
        context, args.control_ip, args.rgb_camera_ip, args.stereo_camera_ip)

    # Saving data for training
    demo_collector = DemoCollector(
        f"{args.demonstrator}_{args.task}_{int(time.time())}.hdf5")

    prev_time = 0
    new_time = 1
    while True:
        # Print fps for debugging
        new_time = time.time()
        fps = 1/(new_time-prev_time)
        print("fps: ", fps)
        prev_time = new_time

        # Wait for control PC to send data (20hz)
        robot_data_pb = input_manager.get_robot_data()

        # Wait for c++ camera script to send data (20hz)
        # Note that since we have conflate=True, we will only get the latest image
        rgb_img = input_manager.get_rgb_img()
        stereo_img = input_manager.get_stereo_img()

        if save_data:
            demo_collector.save_data(robot_data_pb, rgb_img, stereo_img)


if __name__ == "__main__":
    main()
