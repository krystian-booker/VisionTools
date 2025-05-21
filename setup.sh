#!/usr/bin/env bash
set -euo pipefail

REPO_NAME="VisionTools"
LOCALE="en_US.UTF-8"
ROS_KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"

# keep sudo alive
sudo -v
trap 'sudo -v' EXIT

echo "### Locale setup ###"
sudo apt update -y
sudo apt install -y locales
sudo locale-gen "${LOCALE%.*}" "$LOCALE"
sudo update-locale LC_ALL="$LOCALE" LANG="$LOCALE"
export LANG="$LOCALE"
locale

echo "### System update & essentials ###"
# upgrade & essentials in one go
sudo apt upgrade -y
sudo apt install -y git curl software-properties-common
sudo add-apt-repository -y universe

echo "### ROS 2 setup ###"
# add key
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o "$ROS_KEYRING"

# add source
arch=$(dpkg --print-architecture)
codename=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
echo "deb [arch=$arch signed-by=$ROS_KEYRING] \
  http://packages.ros.org/ros2/ubuntu $codename main" \
  | sudo tee "$ROS_LIST" >/dev/null

# final install
sudo apt update -y
sudo apt install -y ros-dev-tools ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "### Done: '$REPO_NAME' ready! ###"
