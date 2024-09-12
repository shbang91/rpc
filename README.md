# Robot Planning and Control (rpc)

[![Build](https://img.shields.io/github/actions/workflow/status/shbang91/rpc/linux.yml?branch=develop)](https://github.com/shbang91/rpc/actions)

rpc is a software framework designed for generating task trajectories (planning) and tracking the trajectories (control) for legged systems.<br/>

Software Framework is developed by Seung Hyeon Bang based on the [PnC](https://github.com/junhyeokahn/PnC) Repository.<br/>

## Dependencies
The controller has been tested on Ubuntu 18.04, Ubuntu 20.04, Ubuntu 23.04, and Mac OSX Ventura. It builds on the shoulders of the following software:<br/>
- [anaconda](https://docs.anaconda.com/anaconda/install/): For Pybullet simulator<br/>
- python dependencies:
```
$ conda env create -f rpc.yml
```
- [conan](https://github.com/conan-io/conan): package manager for C/C++ (for Foxglove)
- [pinocchio](https://github.com/shbang91/pinocchio): rigid body dynamics

###### optional dependencies
- [MatLogger2](https://github.com/shbang91/MatLogger2): logging numeric data (cpp to MAT-files)
- [zmq](https://github.com/shbang91/rpc/blob/main/dependency/scripts/install_zmq.sh): logging numeric data
- [protobuf](https://github.com/shbang91/rpc/blob/main/dependency/scripts/install_protobuf.sh): logging numeric data
- [Foxglove](https://github.com/foxglove): websocket & schema protocols for robot visualization and parameter operations

## Usage for PyBullet
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
python simulator/pybullet/draco_main.py
```
## Foxglove UI (optional)
#### Build
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

#### Run
```
$ conda env create -f visualize.yml
$ conda activate visualize
$ python UI/foxglove/UI_launcher.py --visualizer=foxglove
```
- Access Foxglove server via either:<br/>
  1) [Account in Foxglove webservice](https://app.foxglove.dev/)
  2) [Foxglove application](https://foxglove.dev/download)

- To set up the Foxglove URDF visualizer, please refer to the Readme [in the UI folder](https://github.com/shbang91/rpc/tree/develop/UI/foxglove)

## Meshcat Visualizer (optional)
#### Run
```
$ conda env create -f visualize.yml
$ conda activate visualize
$ python UI/foxglove/UI_launcher.py --visualizer=meshcat
```


## Hardware Usage
- Please refer to this [repository](https://github.com/shbang91/draco3_nodelet) using rpc library
