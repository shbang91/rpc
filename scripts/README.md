## Teleoperation Device
- [Realsense T265](https://www.intelrealsense.com/visual-inertial-tracking-case-study/)

## Installation
Note that from 2.54.X, the codes for T265 are removed, so we need to use the version release v2.53.1 or lower. You can use the below commands for installing the [**Realsense driver**](https://github.com/IntelRealSense/librealsense).
```bash
# Dependencies for the Realsense driver
$ apt-get install libusb-1.0-0-dev xorg-dev libglu1-mesa-dev libglfw3 libglfw3-dev

# Cloning the source code of the Realsense driver
$ git clone https://github.com/IntelRealSense/librealsense.git
$ git checkout v2.53.1

# Setting up authorizing USB devices (You may need "sudo" commands.) 
$ cp config/99-realsense-libusb.rules /etc/udev/rules.d/

# Build and install the driver
$ mkdir build && cd build
$ cmake -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false -DBUILD_PYTHON_BINDINGS=true -DPYTHON_EXECUTABLE=FILEPATH_TO_PYTHON ../
$ make -j4
$ sudo make install
```

Then, set up the conda environment and install the following dependencies
```
$ pip install pyrealsense2==2.53.1.4623
$ pip install pynput
```

## Usage
Run the following script for streaming teleoperation command data.
```
$ python real_teleop.py
```
#### Keyboard Input
```
'r': Start sending teleop commands
'q' or 'esc': Stop sending teleop commands
'p': Close gripper
'o': Open gripper
```
