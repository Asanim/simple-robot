#!/bin/bash -e

# Check for UTF-8
locale

# Install locales and set UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale settings
locale

# Install required packages
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

# Download ROS keyring
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list and install ROS dev tools
sudo apt update && sudo apt install -y ros-dev-tools

# Upgrade existing packages
sudo apt upgrade -y

# Install ROS rolling Desktop
sudo apt install -y ros-rolling-desktop

# Add ROS setup.bash to .bashrc
echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc

# Source ROS setup.bash for the current session
source /opt/ros/rolling/setup.bash

echo "ROS 2 installation complete!"
