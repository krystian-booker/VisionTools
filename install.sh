#!/bin/bash
set -e  # Exit on any error

sudo apt update && sudo apt install locales
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
sudo apt upgrade

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

if ! grep -q "/opt/ros/kilted/setup.bash" "$BASHRC_PATH"; then
    echo "source /opt/ros/kilted/setup.bash" >> "$BASHRC_PATH"
fi

sudo rosdep init   || true
rosdep update

echo "--- Creating ROS 2 Workspace at $HOME_DIR/ros2_ws ---"
ROS_WS="$HOME_DIR/ros2_ws"
mkdir -p "$ROS_WS/src"
cd "$ROS_WS/src"

echo "--- Cloning ROS 2 Nodes…"
git clone -b humble-devel https://github.com/ros-drivers/flir_camera_driver.git
git clone https://github.com/christianrauch/apriltag_ros.git
git clone https://github.com/christianrauch/apriltag_msgs.git

echo "--- Installing dependencies with rosdep…"
cd "$ROS_WS"
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

echo "--- Creating placeholder config files ---"

# NEW: Identity configuration file. This is the MOST IMPORTANT one to edit.
cat <<EOT > "$CONFIG_DIR/robot_identity.yaml"
# FRC Vision System Identity
#
# Edit these values for EACH Orange Pi to give it a unique identity.
# This ensures topics and frames do not conflict on the ROS network.
vision_system:
  ros__parameters:
    # A unique name for this camera setup (e.g., 'cam1', 'cam2', 'front_center_cam').
    # This will be used as the ROS namespace. No slashes.
    camera_name: "cam1"

    # The tf2 frame_id for this camera. This MUST MATCH the link name in your robot's URDF file.
    camera_frame_id: "camera_1_link"
EOT

# Spinnaker camera driver config - this is now mostly for tuning.
cat <<EOT > "$CONFIG_DIR/camera_tuning.yaml"
spinnaker_camera_node:
  ros__parameters:
    serial_number: "0" # <-- CHANGE THIS to your camera's serial number as a string
    acquisition_frame_rate_enable: true
    acquisition_frame_rate: 30.0
    exposure_auto: "Off"
    exposure_time: 2000
    gain_auto: "Off"
    gain: 10.0
    trigger_mode: "Off"
EOT

# AprilTag node config
cat <<EOT > "$CONFIG_DIR/apriltag_tuning.yaml"
apriltag_node:
  ros__parameters:
    family: "tag36h11"
    publish_tag_detections_image: true
EOT

# Set correct ownership for the created directories
sudo chown -R "$EFFECTIVE_USER:$EFFECTIVE_USER" "$HOME_DIR"

# --- 4. Create Smart, Namespaced Launch File ---

echo "--- Creating ROS 2 Launch file for startup ---"
LAUNCH_PKG_DIR="$ROS_WS/src/robot_launch"
mkdir -p "$LAUNCH_PKG_DIR/launch"

# A minimal package.xml is needed for `get_package_share_directory`
cat <<EOT > "$LAUNCH_PKG_DIR/package.xml"
<?xml version="1.0"?>
<package format="3">
  <name>robot_launch</name>
  <version>1.0.0</version>
  <description>Launch files for FRC vision system</description>
  <maintainer email="user@example.com">${EFFECTIVE_USER}</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ros2launch</exec_depend>
</package>
EOT

# A basic CMakeLists.txt
cat <<EOT > "$LAUNCH_PKG_DIR/CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(robot_launch)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch
  DESTINATION share/\${PROJECT_NAME}
)
ament_package()
EOT

# Rebuild the workspace to find the new package
cd "$ROS_WS"
colcon build --symlink-install


# The actual Python launch file - now much more intelligent
cat <<EOT > "$LAUNCH_PKG_DIR/launch/vision_system.launch.py"
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define paths to config files
    home_dir = os.path.expanduser('~')
    identity_config_path = os.path.join(home_dir, 'ros2_config', 'robot_identity.yaml')
    camera_tuning_config = os.path.join(home_dir, 'ros2_config', 'camera_tuning.yaml')
    apriltag_tuning_config = os.path.join(home_dir, 'ros2_config', 'apriltag_tuning.yaml')

    # Load the identity config to get the camera name and frame_id
    with open(identity_config_path, 'r') as f:
        identity_config = yaml.safe_load(f)
        camera_name = identity_config['vision_system']['ros__parameters']['camera_name']
        camera_frame_id = identity_config['vision_system']['ros__parameters']['camera_frame_id']

    # Create a group of actions that will all be pushed into the same namespace
    namespaced_group = GroupAction(
        actions=[
            # Push all nodes into a namespace defined by camera_name
            PushRosNamespace(camera_name),

            # 1. Camera Driver Node
            Node(
                package='spinnaker_camera_driver',
                executable='camera_driver_node',
                name='spinnaker_camera_node',
                parameters=[
                    camera_tuning_config,
                    {'frame_id': camera_frame_id} # Set the frame_id directly
                ],
                output='screen'
            ),

            # 2. Image Processing Node for Undistortion
            # Note: topic remappings are relative to the namespace
            Node(
                package='image_proc',
                executable='image_proc',
                name='image_proc_node',
                remappings=[
                    ('image', 'spinnaker_camera_node/image_raw'),
                    ('image_raw', 'image_proc_node/image_raw_out'), # Avoid topic collision
                    ('image_rect', 'image_rect'), # Output topic
                ],
                output='screen'
            ),
            
            # 3. AprilTag Detection Node
            Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag_node',
                parameters=[apriltag_tuning_config],
                output='screen',
                remappings=[
                    # Subscribe to the *rectified* image from image_proc
                    ('image_rect', 'image_rect'),
                    ('camera_info', 'spinnaker_camera_node/camera_info'),
                ]
            ),
        ]
    )

    return LaunchDescription([namespaced_group])
EOT


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
