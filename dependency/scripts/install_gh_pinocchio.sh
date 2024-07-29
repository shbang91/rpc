#! /bin/bash
PATH_PACKAGE="$(pwd)"

echo '# ==============================================================='
echo "# install Pinocchio for CI pipeline"
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then
    brew install dartsim --only-dependencies
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt-get install build-essential cmake pkg-config git &&
    sudo apt-get install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev &&
    sudo apt-get install libopenscenegraph-dev &&
    sudo apt-get install libbullet-dev &&
    sudo apt-get install libode-dev &&
    sudo apt-get install libtinyxml2-dev &&
    sudo apt-get install liburdfdom-dev &&
    sudo apt-get install libxi-dev libxmu-dev freeglut3-dev &&
    sudo apt-get install libopenscenegraph-dev &&
    sudo apt-get install liboctomap-dev
else
    echo "[error] os not detected"
fi

cd $PATH_PACKAGE/../ &&
git clone --recursive https://github.com/stack-of-tasks/pinocchio &&
cd pinocchio &&
git checkout master &&
mkdir build &&
cd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && make &&
sudo make install


cd ${PATH_PACKAGE}