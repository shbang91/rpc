#! /bin/bash
PATH_PACKAGE="$(pwd)"

echo '# ==============================================================='
echo "# install Pinocchio for CI pipeline"
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then
    brew tap gepetto/homebrew-gepetto
    brew install pinocchio
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    echo '------- Add robotpkg apt repository -------'
    sudo apt install -qqy lsb-release curl
    sudo mkdir -p /etc/apt/keyrings
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
         | sudo tee /etc/apt/keyrings/robotpkg.asc
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
         | sudo tee /etc/apt/sources.list.d/robotpkg.list
    sudo apt update
    echo '------- Install Pinocchio -------'
    sudo apt install -qqy robotpkg-py3*-pinocchio
    echo '------- Configure robotpkg variables -------'
    export PATH=/opt/openrobots/bin:$PATH
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
    export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
    export PYTHONPATH=/opt/openrobots/lib/python3.9/site-packages:$PYTHONPATH # Adapt your desired python version here
    export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
else
    echo "[error] os not detected"
fi

#cd $PATH_PACKAGE/../ &&
#git clone --recursive https://github.com/stack-of-tasks/pinocchio &&
#cd pinocchio &&
#git checkout master &&
#mkdir build &&
#cd build &&
#cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && make &&
#sudo make install


cd ${PATH_PACKAGE}
