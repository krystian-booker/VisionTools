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

echo "### Extras for FLIR/spinnaker ###"
sudo apt install -y \
  ros-jazzy-spinnaker-camera-driver \
  ros-jazzy-flir-camera-description \
  ros-jazzy-flir-camera-msgs \
  ros-jazzy-spinnaker-synchronized-camera-driver

# Build entire workspace
echo "### Building ROS 2 workspace ###"
colcon build --symlink-install

echo "### FLIR camera driver setup ###"
ros2 run spinnaker_camera_driver linux_setup_flir

grep -qxF 'export SPINNAKER_GENTL64_CTI=/opt/ros/${ROS_DISTRO}/lib/spinnaker-gentl/Spinnaker_GenTL.cti' ~/.bashrc \
  || echo 'export SPINNAKER_GENTL64_CTI=/opt/ros/${ROS_DISTRO}/lib/spinnaker-gentl/Spinnaker_GenTL.cti' >> ~/.bashrc

echo "### Self-locating workspace setup ###"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
RC_LINE="source \"${SCRIPT_DIR}/install/setup.bash\""

# If ~/.bashrc doesn’t already contain it, append it:
if ! grep -qxF "$RC_LINE" "$HOME/.bashrc"; then
  echo "$RC_LINE" >> "$HOME/.bashrc"
  echo "→ Added ROS overlay source to ~/.bashrc"
else
  echo "→ ~/.bashrc already sources your workspace"
fi

# temporarily turn off -u so an unset COLCON_TRACE won't abort
set +u
source "${SCRIPT_DIR}/install/setup.bash"
set -u

echo "### Done: '$REPO_NAME' ready! ###"

# Download and install nvm:
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash

# in lieu of restarting the shell
\. "$HOME/.nvm/nvm.sh"

# Download and install Node.js:
nvm install 22

# Verify the Node.js version:
node -v # Should print "v22.16.0".
nvm current # Should print "v22.16.0".

# Verify npm version:
npm -v # Should print "10.9.2".

sudo apt install -y python3-pip
sudo apt install python3.12-venv

# Inside backend directory
# python3 -m venv .venv
# source .venv/bin/activate
# pip install -r requirements.txt

#Oak-D
# sudo wget -qO- https://docs.luxonis.com/install_depthai.sh | bash
