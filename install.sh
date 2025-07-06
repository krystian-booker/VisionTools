#!/bin/bash
set -e  # Exit on any error

# --- 1. System and ROS 2 Prerequisite Installation ---
echo "--- Updating package lists and installing base dependencies ---"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

echo "--- Setting up ROS 2 apt sources ---"
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

echo "--- Installing ROS 2 and development tools ---"
sudo apt update
sudo apt -y upgrade
sudo apt install -y ros-kilted-desktop ros-dev-tools ros-kilted-image-proc
sudo apt-get install -y git wget curl build-essential python3-pip python3-rosdep

# --- FLIR Group Setup ---
echo "--- Creating and configuring flirimaging group ---"
# Use '|| true' to prevent script failure if the group already exists
sudo addgroup --system flirimaging || true
sudo usermod -a -G flirimaging ${USER}

# --- ADDED: udev Rules for FLIR/Spinnaker ---
echo "--- Setting up udev rules for FLIR cameras ---"
# This creates/overwrites the rules file to ensure it's correct
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", GROUP="flirimaging"' | sudo tee /etc/udev/rules.d/40-flir-spinnaker.rules > /dev/null
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1724", GROUP="flirimaging"' | sudo tee -a /etc/udev/rules.d/40-flir-spinnaker.rules > /dev/null
echo "--- Reloading udev rules ---"
sudo service udev restart
sudo udevadm trigger

# --- GRUB Configuration for USB Memory ---
echo "--- Updating GRUB configuration for USB buffer size ---"
# Check if the setting is already present
if ! grep -q "usbcore.usbfs_memory_mb=1000" /etc/default/grub; then
    # Use sed to replace the line and then update grub
    sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"/' /etc/default/grub
    sudo update-grub
    echo "GRUB configuration has been updated."
else
    echo "GRUB configuration for usbcore.usbfs_memory_mb already set. No changes made."
fi

# --- 2. Workspace and Environment Setup ---
HOME_DIR="$HOME"
BASHRC_PATH="$HOME_DIR/.bashrc"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROS_WS="$SCRIPT_DIR"

echo "--- Sourcing ROS 2 environment in .bashrc ---"
if ! grep -q "/opt/ros/kilted/setup.bash" "$BASHRC_PATH"; then
    echo "source /opt/ros/kilted/setup.bash" >> "$BASHRC_PATH"
fi

echo "--- Initializing rosdep ---"
sudo rosdep init || true
rosdep update

echo "--- Adding Spinnaker GenTL environment variable to .bashrc ---"
if ! grep -q "export SPINNAKER_GENTL64_CTI" "$BASHRC_PATH"; then
    echo 'export SPINNAKER_GENTL64_CTI=/opt/ros/${ROS_DISTRO}/lib/spinnaker-gentl/Spinnaker_GenTL.cti' >> "$BASHRC_PATH"
fi

# --- 3. Setup Local Configuration Files (BEFORE BUILD) ---
echo "--- Creating local configuration directory at $ROS_WS/config ---"
CONFIG_DIR="$ROS_WS/config"
EXAMPLES_DIR="$ROS_WS/examples"
mkdir -p "$CONFIG_DIR"

echo "--- Copying example configuration files to local configs ---"
# This creates the user-editable config files from the version-controlled examples.
cp "$EXAMPLES_DIR/robot_identity.yaml" "$CONFIG_DIR/robot_identity.yaml"
cp "$EXAMPLES_DIR/camera_tuning.yaml" "$CONFIG_DIR/camera_tuning.yaml"
cp "$EXAMPLES_DIR/apriltag_tuning.yaml" "$CONFIG_DIR/apriltag_tuning.yaml"

# --- 4. Build the ROS 2 Workspace ---
echo "--- Setting up ROS 2 Workspace at $ROS_WS ---"
cd "$ROS_WS" # Make sure we are in the repo root
source /opt/ros/kilted/setup.bash

echo "--- Installing ROS dependencies with rosdep ---"
rosdep install --from-paths src --ignore-src -r -y

echo "--- Building Workspace with colcon ---"
# This will now succeed because the config/ directory exists and CMake can find it.
colcon build --symlink-install

echo "--- Sourcing local workspace in .bashrc ---"
if ! grep -q "$ROS_WS/install/setup.bash" "$BASHRC_PATH"; then
    echo "source $ROS_WS/install/setup.bash" >> "$BASHRC_PATH"
fi

# --- Final Message ---
echo "----------------------------------------------------------------"
echo "Script finished!"
echo "Please run 'sudo reboot' now."
echo "----------------------------------------------------------------"