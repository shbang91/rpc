# Robot Planning, Control, and Deployment (rpc)

[![Build](https://img.shields.io/github/actions/workflow/status/shbang91/rpc/linux.yml?branch=develop)](https://github.com/shbang91/rpc/actions)

**RPC** is a Modular Framework for Robot Planning, Control, and Deployment. It is designed to integrate multiple physics-based simulators, planning and control modules, visualization tools, plotting and logging utilities, and operator interfaces for robotic systems.<br/>
<img src="docs/images/draco_example.gif" width="100%"/>
If you find our work useful in your research, please consider the following [citation](#book-citation).

## :package: Mandatory Dependencies
The controller has been tested on Ubuntu 18.04, Ubuntu 20.04, Ubuntu 22.04, and Mac OSX Sonoma. It builds on the shoulders of the following software:<br/>
- [anaconda](https://docs.anaconda.com/anaconda/install/): For Pybullet simulator<br/>
- python dependencies:
```
$ conda env create -f rpc.yml
```
- [pinocchio](https://github.com/shbang91/pinocchio): Rigid body dynamics library

## :newspaper_roll: Optional Dependencies
#### :walking: MPC for Locomotion
- [hpipm-cpp](https://github.com/shbang91/hpipm-cpp): C++ wrapper for HPIPM (QP solver). Note that **blasfo**, **hpipm**, and **hpipm-cpp** wrapper should be installed
#### :pinching_hand: Teleoperation for Manipulation
- [Teleoperation](https://github.com/shbang91/rpc/tree/develop/scripts): Please follow the instructions for installation and usage
#### :toolbox: Utilities for Visualization, Plotting, Logging and Operator Interfaces
- [MatLogger2](https://github.com/shbang91/MatLogger2): logging numeric data (cpp to MAT-files)
- [zmq](https://github.com/shbang91/rpc/blob/main/dependency/scripts/install_zmq.sh): socket communication protocol
- [protobuf](https://github.com/shbang91/rpc/blob/main/dependency/scripts/install_protobuf.sh): structured data serialization
- [conan](https://github.com/conan-io/conan): package manager for C/C++ (for Foxglove)
- [Foxglove](https://github.com/foxglove): websocket & schema protocols for robot visualization and parameter operations


## :computer: Usage 
#### (1) PyBullet
- Source conda environment:<br/>
```
$ conda activate rpc
```
- Compile:<br/>
```
$ mkdir build && cd build
$ cmake ..
$ make -j4
```
- Run simulation:<br/>
```
$ python simulator/pybullet/draco_main.py
```
#### (2) MuJoCo
- Compile:<br/>
```
$ mkdir build && cd build
$ cmake ..
$ make -j4
```
- Run simulation:<br/>
```
$ ./bin/run_draco
```
#### Keyboard Input
- Please see the [example implementation](https://github.com/shbang91/rpc/blob/develop/controller/draco_controller/draco_interrupt_handler.cpp) for DRACO 3 

## :tv: Visualization
#### (1) Foxglove UI (optional)
###### Build
- Source conda environment:<br/>
```
$ conda activate rpc
```
- Compile:<br/>
```
$ mkdir -p ~/.conan2/profiles/ && cp .github/conan_profile ~/.conan2/profiles/default
$ conan install conanfile.txt --build=missing
$ cd build
$ cmake .. -DBUILD_WITH_ZMQ_PROTOBUF=ON -DBUILD_WITH_FOXGLOVE=ON
$ make -j4
```

###### Run
```
$ conda env create -f visualize.yml
$ conda activate visualize
$ python UI/foxglove/UI_launcher.py --visualizer=foxglove
```
- Access Foxglove server via either:<br/>
  1) [Account in Foxglove webservice](https://app.foxglove.dev/)
  2) [Foxglove application](https://foxglove.dev/download)

- To set up the Foxglove URDF visualizer, please refer to the Readme [in the UI folder](https://github.com/shbang91/rpc/tree/develop/UI/foxglove)

#### (2) Meshcat Visualizer (optional)
###### Run
```
$ conda env create -f visualize.yml
$ conda activate visualize
$ python UI/foxglove/UI_launcher.py --visualizer=meshcat
```


## :robot: Hardware Usage
- Please refer to this [repository](https://github.com/shbang91/draco3_nodelet) using rpc library

## :book: Citation
```
@misc{bang2024rpc,
    title={RPC: A Modular Framework for Robot Planning, Control, and Deployment},
    author={Bang, Seung Hyeon and Gonzalez, Carlos and Moore, Gabriel and Kang, Dong Ho
            and Seo, Mingyo and Sentis, Luis},
    year={2024}
    eprint={XXXX.XXXXXX},
    archivePrefix={arXiv},
    primaryClass={cs.RO},
    url={}
}
```
