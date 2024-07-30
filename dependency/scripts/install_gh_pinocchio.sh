#! /bin/bash
PATH_PACKAGE="$(pwd)"

echo '# ==============================================================='
echo "# install Pinocchio for CI pipeline"
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then
    brew tap gepetto/homebrew-gepetto
    brew install pinocchio
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt install -qqy lsb-release curl
    sudo mkdir -p /etc/apt/keyrings
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
         | sudo tee /etc/apt/keyrings/robotpkg.asc
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
         | sudo tee /etc/apt/sources.list.d/robotpkg.list
    sudo apt update
    sudo apt install -qqy robotpkg-py3*-pinocchio
else
    echo "[error] os not detected"
fi

cd ${PATH_PACKAGE}
