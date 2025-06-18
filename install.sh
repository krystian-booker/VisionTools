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

# --- 2. ROS 2 Workspace Setup ---
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

echo "--- Setting up ROS 2 Workspace at $ROS_WS ---"
cd "$ROS_WS" # Make sure we are in the repo root
source /opt/ros/kilted/setup.bash

echo "--- Installing ROS dependencies with rosdep ---"
rosdep install --from-paths src --ignore-src -r -y

echo "--- Building Workspace with colcon ---"
colcon build --symlink-install

echo "--- Sourcing local workspace in .bashrc ---"
if ! grep -q "$ROS_WS/install/setup.bash" "$BASHRC_PATH"; then
    echo "source $ROS_WS/install/setup.bash" >> "$BASHRC_PATH"
fi

# --- 3. Setup Local Configuration Files ---
echo "--- Creating local configuration directory at $ROS_WS/config ---"
CONFIG_DIR="$ROS_WS/config"
EXAMPLES_DIR="$ROS_WS/examples"
mkdir -p "$CONFIG_DIR"

echo "--- Copying example configuration files to local configs ---"
cp "$EXAMPLES_DIR/robot_identity.yaml" "$CONFIG_DIR/robot_identity.yaml"
cp "$EXAMPLES_DIR/camera_tuning.yaml" "$CONFIG_DIR/camera_tuning.yaml"
cp "$EXAMPLES_DIR/apriltag_tuning.yaml" "$CONFIG_DIR/apriltag_tuning.yaml"


# --- 4. Create and Enable systemd Service ---
# The EFFECTIVE_USER variable ensures the service runs as the user who ran the script, not as root
EFFECTIVE_USER=${SUDO_USER:-$USER}

echo "--- Creating systemd service file ---"
SERVICE_FILE="/etc/systemd/system/ros2-vision.service"

sudo bash -c "cat > $SERVICE_FILE" <<EOT
[Unit]
Description=ROS 2 Vision System for FRC
After=network.target

[Service]
User=$EFFECTIVE_USER
WorkingDirectory=$ROS_WS
ExecStart=/bin/bash -c "source /opt/ros/kilted/setup.bash && source $ROS_WS/install/setup.bash && ros2 launch robot_launch vision_system.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOT

echo "--- Enabling systemd service ---"
sudo systemctl daemon-reload
sudo systemctl enable ros2-vision.service

# --- Final Instructions ---
echo ""
echo "--------------------------------------------------------"
echo "---          Setup Complete!                         ---"
echo "--------------------------------------------------------"
echo ""
echo "CRITICAL NEXT STEPS:"
echo ""
echo "1. PREPARE YOUR GIT REPOSITORY:"
echo "   The script created a 'config/' directory for your local settings."
echo "   >> You MUST add this directory to your .gitignore file! <<"
echo "   Run the following command from your repository root:"
echo "   echo 'config/' >> .gitignore"
echo ""
echo "2. EDIT YOUR LOCAL CONFIGS:"
echo "   The script created local configuration files in '$CONFIG_DIR/'."
echo "   You MUST edit these files to match your robot's hardware."
echo "   - In 'robot_identity.yaml', set a unique 'camera_name'."
echo "   - In 'camera_tuning.yaml', set the correct 'serial_number'."
echo ""
echo "3. VERIFY LAUNCH FILE (One-time check):"
echo "   Your launch file 'vision_system.launch.py' must know where to find these configs."
echo "   Make sure it is looking for them in the '$CONFIG_DIR/' directory."
echo ""
echo "4. REBOOT:"
echo "   A reboot is required for all changes to take effect."
echo "   Run 'sudo reboot'"
echo ""
echo "After rebooting, the ROS 2 nodes will start automatically."
echo "Check status with: 'systemctl status ros2-vision.service'"
echo ""