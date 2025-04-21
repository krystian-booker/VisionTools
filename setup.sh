# Ask for sudo up front
sudo -v

echo "Installing all requirements..."
sudo apt update

# VSCode
sudo apt-get install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
rm -f packages.microsoft.gpg

sudo apt install apt-transport-https
sudo apt update
sudo apt install code

# Set local
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#Python
sudo apt install -y software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.10 python3.10-venv python3.10-distutils
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1
sudo apt install -y python3-colcon-common-extensions

#AprilTag3 dependencies
sudo apt install build-essential cmake git
sudo apt install libopencv-dev

#ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash

echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc

#ROS2 camera support
sudo apt update
sudo apt install ros-humble-v4l2-camera ros-humble-image-transport ros-humble-cv-bridge

#ROS2 GTSAM
sudo apt install ros-$ROS_DISTRO-gtsam

#Setup ROS2 workspace
git clone --recurse-submodules https://github.com/krystian-booker/ROS2FRC.git
cd ROS2FRC

# AprilTag3 build
cd src
cd apriltag
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig

#apriltag_msgs
cd ..
cd .. #should be back in src folder now
cd apriltag_msgs
colcon build
source install/setup.bash
source "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/install/setup.bash"

#apriltag_ros
cd .. #should be back in src folder now
cd apriltag_ros
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
source "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/install/setup.bash"