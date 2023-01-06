# Robot Planning and Control (rpc)
rpc is a software framework designed for generating task trajectories (planning) and tracking the trajectories (control) for legged systems.<br/>

Software Framework is developed by Seung Hyeon Bang based on the [PnC](https://github.com/junhyeokahn/PnC) Repository.<br/>

## Dependencies
The controller has been tested on Ubuntu 18.04 and Mac OSX Ventura. It builds on the shoulders of the following software:<br/>
- [anaconda](https://docs.anaconda.com/anaconda/install/): For Pybullet simulator<br/>
- python dependencies: 
```
conda env create -f rpc.yml
```
- [pinocchio](https://github.com/shbang91/pinocchio): rigid body dynamics
- [zmq](https://github.com/zeromq/cppzmq): logging numeric data
- [protobuf](https://github.com/protocolbuffers/protobuf): logging numeric data

###### optional dependencies
- [MatLogger2](https://github.com/ADVRHumanoids/MatLogger2/tree/devel): logging numeric data (cpp to MAT-files)

## Usage
- source conda environment:<br/>
```
$ conda activate rpc
```
- Compile:<br/>
```
$ mkdir build
$ cd build 
$ cmake ..
$ make -j4
```
- Run simulation:<br/>
```
python simulator/pybullet/draco_main.py
```
###### Hardware Usage
- Please refer to this [repository](https://github.com/shbang91/draco3_nodelet) using rpc library
