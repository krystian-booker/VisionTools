#!/bin/bash
set -e

# Ask for sudo up front
sudo -v

echo "Installing all requirements..."
sudo apt update && sudo apt upgrade -y

####################################
# Essentials
####################################
sudo apt install -y wget curl gpg apt-transport-https software-properties-common build-essential cmake git locales

####################################
# Set locale
####################################
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

####################################
# Install VSCode
####################################
echo "Installing VSCode..."
if ! command -v code &> /dev/null; then
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
    sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
    rm microsoft.gpg
    sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
    sudo apt update
    sudo apt install -y code
fi

####################################
# Install ROS
####################################
echo "Installing ROS Noetic..."

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Source ROS now for this script to use its commands
source /opt/ros/noetic/setup.bash

# Add to bashrc for future terminals
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y ros-noetic-catkin

####################################
# Install USB Camera Driver
####################################
sudo apt install -y ros-noetic-usb-cam ros-noetic-camera-info-manager ros-noetic-diagnostic-updater ros-noetic-dynamic-reconfigure ros-noetic-image-exposure-msgs ros-noetic-image-transport ros-noetic-nodelet ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-wfov-camera-msgs

####################################
# Install additional camera drivers
####################################
if [ -f ./camera_install.sh ]; then
    echo "Found camera_install.sh, running it first..."
    chmod +x ./camera_install.sh
    ./camera_install.sh
fi

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

####################################
# Install TagSLAM requirements
####################################
sudo apt install python3-vcstool
sudo apt install python3-catkin-tools python3-osrf-pycommon # Ubuntu >20.04

if dpkg -l | grep -q "^ii  gtsam "; then
    sudo apt remove -y gtsam
fi

sudo add-apt-repository --remove ppa:bernd-pfrommer/gtsam
sudo apt-add-repository --remove ppa:borglab/gtsam-release-4.0
sudo apt-add-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

cd ~/ROS2FRC/
vcs import --recursive . < src/tagslam_root/tagslam_root.repos
rosdep install --from-paths src --ignore-src -r -y

# Working TagSLAM build
catkin clean -y
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBoost_NO_BOOST_CMAKE=ON 
catkin build
