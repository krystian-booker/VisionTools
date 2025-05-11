#!/bin/bash
set -e

# Name of your workspace/repo directory
REPO_NAME="ROS2FRC"

# Ask for sudo up front
sudo -v

echo "Installing all requirements..."
sudo apt update && sudo apt upgrade -y

####################################
# Essentials
####################################
sudo apt install -y wget curl gpg apt-transport-https software-properties-common build-essential cmake git locales nginx

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
sudo apt install -y ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    ros-noetic-catkin ros-noetic-usb-cam ros-noetic-camera-info-manager ros-noetic-diagnostic-updater ros-noetic-dynamic-reconfigure \
    ros-noetic-image-exposure-msgs ros-noetic-image-transport ros-noetic-nodelet ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-wfov-camera-msgs \
    python3-vcstool python3-catkin-tools python3-osrf-pycommon \
    autoconf automake nano libeigen3-dev libboost-all-dev libsuitesparse-dev doxygen \
    libopencv-dev libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
    python3-dev python3-pip python3-scipy python3-matplotlib ipython3 python3-wxgtk4.0 \
    python3-tk python3-igraph python3-pyx \
    libgoogle-glog-dev libgflags-dev libglew-dev

# Source ROS now for this script to use its commands
source /opt/ros/noetic/setup.bash

# Add to bashrc for future terminals
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

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
cd ~/"$REPO_NAME"/
vcs import --recursive . < src/tagslam_root/tagslam_root.repos   # TagSLAM
# (Kalibr is already present as a git sub-module inside src)

####################################
# Resolve ROS dependencies
####################################
rosdep install --from-paths src --ignore-src -r -y

####################################
# Configure and build the whole workspace (TagSLAM + Kalibr)
####################################
catkin config --merge-devel --extend /opt/ros/noetic \
              -DCMAKE_BUILD_TYPE=Release \
              -DBoost_NO_BOOST_CMAKE=ON
catkin build -j"$(nproc)"

# Source the workspace so Kalibr is on the ROS path for this shell
source devel/setup.bash

# make TagSLAM/Kalibr workspace available in every new shell
WORKSPACE_SETUP="source \$HOME/$REPO_NAME/devel/setup.bash"
if ! grep -Fxq "$WORKSPACE_SETUP" "$HOME/.bashrc"; then
    echo "$WORKSPACE_SETUP" >> "$HOME/.bashrc"
fi

echo "✅  TagSLAM and Kalibr have been built successfully!"

# roslaunch flir_camera_node flir_cameras.launch

# Setup Nginx to serve $REPO_NAME/web
echo "Configuring Nginx to serve ~/$REPO_NAME/web ..."
NGINX_CONF="/etc/nginx/sites-available/$REPO_NAME"
sudo tee "${NGINX_CONF}" > /dev/null <<EOF
server {
    listen 80 default_server;
    listen [::]:80 default_server;
    root \$HOME/$REPO_NAME/web;
    index index.html index.htm;
    server_name _;
    location / {
        try_files \$uri \$uri/ =404;
    }
}
EOF

# Enable our site, disable default, reload
sudo ln -sf "${NGINX_CONF}" /etc/nginx/sites-enabled/$REPO_NAME
sudo rm -f /etc/nginx/sites-enabled/default
sudo nginx -t
sudo systemctl reload nginx

# Ensure the web directory is world-readable
chmod -R o+rx "$HOME/$REPO_NAME/web"

echo "✅  Nginx is now serving your web UI at http://<this-machine-IP>/"