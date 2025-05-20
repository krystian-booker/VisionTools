#!/bin/bash
set -e

# Name of your workspace/repo directory
REPO_NAME="VisionTools"

# Ask for sudo up front
sudo -v

echo "Installing all requirements..."
sudo apt update && sudo apt upgrade -y

####################################
# Essentials
####################################
sudo apt install -y git
