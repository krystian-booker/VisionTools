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
export LANG=en_US.UTF-8

####################################
# Install VSCode
####################################
echo "Installing VSCode..."
if ! command -v code &> /dev/null; then
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/packages.microsoft.gpg > /dev/null
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
    sudo apt update
    sudo apt install -y code
fi

####################################
# Install Python
####################################
echo "Installing Python..."
if ! python3.10 --version &> /dev/null; then
    sudo add-apt-repository -y ppa:deadsnakes/ppa
    sudo apt update
    sudo apt install -y python3.10 python3.10-venv python3.10-distutils
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1
fi

####################################
# AprilTag3 dependencies
####################################
sudo apt install -y libopencv-dev

####################################
# Install ROS 2 Humble
####################################
echo "Installing ROS2 Humble..."
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo add-apt-repository universe -y
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
                    ros-humble-v4l2-camera ros-humble-image-transport ros-humble-cv-bridge \
                    python3-rosdep

# Add ROS to bashrc if not already there
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# GTSAM
sudo apt install -y ros-$ROS_DISTRO-gtsam

# Force shell rehash
hash -r

####################################
# Setup ROS2 workspace
####################################
echo "Setting up ROS2 workspace..."
WORKSPACE_DIR=~/ROS2FRC

# Clone workspace if it doesn't exist
if [ ! -d "$WORKSPACE_DIR" ]; then
    git clone --recurse-submodules https://github.com/krystian-booker/ROS2FRC.git "$WORKSPACE_DIR"
fi

# Build AprilTag3
echo "Building AprilTag3..."
cd "$WORKSPACE_DIR/src/apriltag" || exit 1
mkdir -p build && cd build || exit 1
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig

# Source workspace setup file helper
source_workspace() {
    local setup_file="$1/install/setup.bash"
    if [ -f "$setup_file" ]; then
        source "$setup_file"
    else
        echo "Warning: Could not find setup file at $setup_file"
    fi
}

# Back to workspace root
cd "$WORKSPACE_DIR" || exit 1

# Install dependencies and build full workspace
echo "Resolving dependencies with rosdep..."
sudo rosdep init 2>/dev/null || echo "rosdep already initialized"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "Building full workspace..."
colcon build --symlink-install
