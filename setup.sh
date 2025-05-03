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
# Build & Python prerequisites (TagSLAM + Kalibr)
####################################
sudo apt-get install -y python3-vcstool python3-catkin-tools python3-osrf-pycommon \
  autoconf automake nano libeigen3-dev libboost-all-dev libsuitesparse-dev doxygen \
  libopencv-dev libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
  python3-dev python3-pip python3-scipy python3-matplotlib ipython3 python3-wxgtk4.0 \
  python3-tk python3-igraph python3-pyx \
  libgoogle-glog-dev libgflags-dev libglew-dev

####################################
# TagSLAM setup
####################################
# Remove existing GTSAM if installed
if dpkg -l | grep -q "^ii  gtsam "; then
    sudo apt-get remove -y gtsam
fi

# Remove old PPAs (ignore errors if they don't exist)
sudo add-apt-repository --remove ppa:bernd-pfrommer/gtsam -y || true
sudo add-apt-repository --remove ppa:borglab/gtsam-release-4.0 -y || true

# Add correct PPA
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.1

# Update package lists and install GTSAM
sudo apt-get update
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev

####################################
# Clone/update repositories
####################################
cd ~/ROS2FRC/
vcs import --recursive . < src/tagslam_root/tagslam_root.repos   # TagSLAM
# (Kalibr is already present as a git sub-module inside src)

####################################
# Resolve ROS dependencies
####################################
rosdep install --from-paths src --ignore-src -r -y

####################################
# Configure and build the whole workspace (TagSLAM + Kalibr)
####################################
catkin config --install --extend /opt/ros/noetic \
              --cmake-args -DCMAKE_BUILD_TYPE=Release \
                           -DBoost_NO_BOOST_CMAKE=ON
catkin build -j"$(nproc)"

echo "source \$HOME/ROS2FRC/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "✅  TagSLAM and Kalibr have been built successfully!"
