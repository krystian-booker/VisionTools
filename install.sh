#!/bin/bash
set -e  # Exit on any error

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install -y ros-dev-tools

sudo apt update
sudo apt -y upgrade

sudo apt install -y ros-kilted-desktop

sudo apt-get install -y \
    git \
    wget \
    curl \
    build-essential \
    python3-pip \
    python3-rosdep \
    ros-dev-tools

sudo apt-get install -y ros-kilted-image-proc

HOME_DIR="$HOME"
BASHRC_PATH="$HOME_DIR/.bashrc"

SCRIPT_DIR="$( cd "$( dirname "\${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROS_WS="$SCRIPT_DIR"

if ! grep -q "/opt/ros/kilted/setup.bash" "$BASHRC_PATH"; then
    echo "source /opt/ros/kilted/setup.bash" >> "$BASHRC_PATH"
fi

sudo rosdep init   || true
rosdep update

echo "--- Setting up ROS 2 Workspace at $ROS_WS ---"

# Source directory 'src' is assumed to be part of the repo.
# Cloning is no longer done by this script. Dependencies are expected to be in 'src'.

echo "--- Installing dependencies with rosdep…"
cd "$ROS_WS" # Make sure we are in the repo root for rosdep and colcon
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src -r -y

echo "--- Building Workspace with colcon ---"
colcon build --symlink-install

# Add workspace sourcing to .bashrc
if ! grep -q "$ROS_WS/install/setup.bash" "$BASHRC_PATH"; then
    echo "source $ROS_WS/install/setup.bash" >> "$BASHRC_PATH"
fi

# Add workspace sourcing to .bashrc
if ! grep -q "$ROS_WS/install/setup.bash" "$BASHRC_PATH"; then
    echo "source $ROS_WS/install/setup.bash" >> "$BASHRC_PATH"
fi


# --- 3. Setup Configuration Directory and Files ---
echo "--- Creating configuration directory at $HOME_DIR/ros2_config ---"
CONFIG_DIR="$HOME_DIR/ros2_config"
mkdir -p "$CONFIG_DIR"

echo "--- Copying configuration files ---"
cp "$SCRIPT_DIR/examples/robot_identity.yaml" "$CONFIG_DIR/robot_identity.yaml"
cp "$SCRIPT_DIR/examples/camera_tuning.yaml" "$CONFIG_DIR/camera_tuning.yaml"
cp "$SCRIPT_DIR/examples/apriltag_tuning.yaml" "$CONFIG_DIR/apriltag_tuning.yaml"

# Set correct ownership for the created directories
sudo chown -R "$EFFECTIVE_USER:$EFFECTIVE_USER" "$HOME_DIR"

# --- 4. Create Smart, Namespaced Launch File ---

echo "--- Creating ROS 2 Launch file for startup ---"
LAUNCH_PKG_DIR="$ROS_WS/src/robot_launch"
mkdir -p "$LAUNCH_PKG_DIR/launch"

echo "--- Copying vision_system.launch.py for robot_launch ---"
cp "$SCRIPT_DIR/src/robot_launch/launch/vision_system.launch.py" "$LAUNCH_PKG_DIR/launch/vision_system.launch.py"

echo "--- Copying package.xml for robot_launch ---"
cp "$SCRIPT_DIR/src/robot_launch/package.xml" "$LAUNCH_PKG_DIR/package.xml"

echo "--- Copying CMakeLists.txt for robot_launch ---"
cp "$SCRIPT_DIR/src/robot_launch/CMakeLists.txt" "$LAUNCH_PKG_DIR/CMakeLists.txt"

# package.xml is now expected to be in the repository at src/robot_launch/package.xml
# and will be copied into $LAUNCH_PKG_DIR by a new command.

# Rebuild the workspace to find the new package
cd "$ROS_WS"
colcon build --symlink-install

# The Python launch file vision_system.launch.py is now expected to be
# in the repository at src/robot_launch/launch/vision_system.launch.py
# and will be copied into $LAUNCH_PKG_DIR/launch by a new command.

# --- 5. Create and Enable systemd Service ---

echo "--- Creating systemd service file ---"
SERVICE_FILE="/etc/systemd/system/ros2-vision.service"

sudo bash -c "cat > $SERVICE_FILE" <<EOT
[Unit]
Description=ROS 2 Vision System for FRC
After=network.target

[Service]
User=$EFFECTIVE_USER
WorkingDirectory=$HOME_DIR
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
echo "---                 Setup Complete!                  ---"
echo "--------------------------------------------------------"
echo ""
echo "CRITICAL NEXT STEPS:"
echo ""
echo "1. EDIT YOUR IDENTITY CONFIG:"
echo "   The script created '$CONFIG_DIR/robot_identity.yaml'."
echo "   >> You MUST edit this file on each Pi to set a unique 'camera_name' << "
echo "   >> and the correct 'camera_frame_id' to match your URDF.        << "
echo ""
echo "2. EDIT YOUR TUNING CONFIG:"
echo "   You MUST also edit '$CONFIG_DIR/camera_tuning.yaml' and set the correct 'serial_number' for this Pi's camera."
echo ""
echo "3. REBOOT:"
echo "   A reboot is required for all changes to take effect."
echo "   Run 'sudo reboot'"
echo ""
echo "After rebooting, the ROS 2 nodes will start automatically in their own namespace."
echo "You can check the status of the service with: 'systemctl status ros2-vision.service'"
echo "To see topic names, run 'ros2 topic list'. You should see topics like '/cam1/image_rect'."
echo ""
