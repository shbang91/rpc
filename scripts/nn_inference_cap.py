# -*- coding: utf-8 -*- 
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
import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
    
from scripts.nn_wrapper import NNWrapper, ReplayAction
from scripts.demo_collector import DemoCollector, convert_protobuf_and_image
from scripts.input_data_manager import ReplayInputDataManager, SocketInputDataManager
from scripts.output_data_manager import OutputDataManager


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control_ip", type=str,
                        default="tcp://192.168.1.184:5557")
    parser.add_argument("--rgb_camera_ip", type=str,
                        default="tcp://localhost:5557")
    parser.add_argument("--stereo_camera_ip", type=str,
                        default="tcp://localhost:5558")
    parser.add_argument("--demonstrator", type=str, default="steve")
    parser.add_argument("--task", type=str, default="box")
    parser.add_argument("--nn_path", type=str, default="models/nn_model.h5")
    parser.add_argument("--obs_replay_path", type=str, default="", help="if specified, will use the replay file instead of the zmq socket")
    parser.add_argument("--actions_replay_path", type=str, default="", help="if specified, will use the replay file instead of the NN")
    args = parser.parse_args()

    # Socket initialize
    context = zmq.Context()
    if (args.obs_replay_path != ""):
        input_manager = ReplayInputDataManager(args.obs_replay_path)
    else:
        input_manager = SocketInputDataManager(
        context, args.control_ip, args.rgb_camera_ip, args.stereo_camera_ip)

    # Executing the policy
    output_manager = OutputDataManager(context)
    nn_offline = ReplayAction(args.actions_replay_path)
    nn_policy = NNWrapper(args.nn_path)

    input("Press Enter to start recorded actions")
    nn = nn_offline
    offline_flag = True

    prev_time = 0
    new_time = 1

    # Initialize state
    robot_data_pb = input_manager.get_robot_data()
    rgb_img = input_manager.get_rgb_img()
    stereo_img = input_manager.get_stereo_img()
    nn.reset(convert_protobuf_and_image(robot_data_pb, rgb_img, stereo_img))
        
    while True:
        # Print fps for debugging
        new_time = time.time()
        fps = 1/(new_time-prev_time)
        print("fps: ", fps)
        prev_time = new_time

        zmq_start = time.perf_counter()
        # Wait for control PC to send data (20hz)
        robot_data_pb = input_manager.get_robot_data()
        robot_end = time.perf_counter()

        # Wait for c++ camera script to send data (20hz)
        # Note that since we have conflate=True, we will only get the latest image
        rgb_img = input_manager.get_rgb_img()
        rgb_end = time.perf_counter()
        stereo_img = input_manager.get_stereo_img()
        stereo_end = time.perf_counter()
        zmq_end = time.perf_counter()
        #print("robot time: ", robot_end-zmq_start)
        #print("rgb time: ", rgb_end-robot_end)
        #print("stereo time: ", stereo_end-rgb_end)
        #print("zmq total time: ", zmq_end-zmq_start)

        print(str(robot_end-zmq_start) + ", " + str(rgb_end-robot_end) + ", " + str(stereo_end-rgb_end))
        # time this line
        nn_start = time.perf_counter()
        actions = nn.forward(convert_protobuf_and_image(robot_data_pb, rgb_img, stereo_img))
        nn_end = time.perf_counter()
        #print("nn time: ", nn_end-nn_start)

        if offline_flag and actions['r_gripper']==1:

            input("Press Enter to start executing policy")
            offline_flag = False
            actions['r_gripper'] = 0

            nn = nn_policy
            # Initialize state
            robot_data_pb = input_manager.get_robot_data()
            rgb_img = input_manager.get_rgb_img()
            stereo_img = input_manager.get_stereo_img()
            nn.reset(convert_protobuf_and_image(robot_data_pb, rgb_img, stereo_img)) 

        output_manager.send(actions)
        send_end = time.perf_counter()
        #print("send time: ", send_end-nn_end)
        print(str(robot_end-zmq_start) + ", " + str(rgb_end-robot_end) + ", " + str(stereo_end-rgb_end) + ", " + str(nn_end-nn_start) + ", " + str(send_end - nn_end))


if __name__ == "__main__":
    main()
