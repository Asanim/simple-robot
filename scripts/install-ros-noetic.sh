#!/bin/bash

# Check if the script is being run as root
if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root."
    exit 1
fi

# Add ROS repository and key
echo "Adding ROS repository..."
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-get update
apt-get install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS melodic
echo "Installing ROS melodic..."
apt-get update
apt-get install -y ros-melodic-desktop-full

# Initialize rosdep
echo "Initializing rosdep..."
rosdep init
rosdep update

# Setting up environment variables
echo "Setting up environment variables..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional tools and dependencies
echo "Installing additional tools and dependencies..."
apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

echo "ROS melodic installation complete."

# Install freenect and turtlebot packages
echo "Installing Freenect and TurtleBot packages..."
apt-get install -y ros-melodic-freenect-launch ros-melodic-turtlebot* freenect

# Install Arduino IDE and rosserial dependencies
echo "Installing Arduino IDE and rosserial dependencies..."
apt-get install -y arduino

# Install rosserial for Arduino
echo "Installing rosserial for Arduino..."
apt-get install -y ros-melodic-rosserial-arduino ros-melodic-rosserial

# Install rosserial_arduino package
echo "Installing rosserial_arduino package..."
apt-get install -y ros-melodic-rosserial-arduino

# Install Arduino IDE and rosserial library
echo "Installing Arduino IDE and rosserial library..."
wget https://downloads.arduino.cc/arduino-1.8.15-linux64.tar.xz
tar -xvf arduino-1.8.15-linux64.tar.xz
rm arduino-1.8.15-linux64.tar.xz
mv arduino-1.8.15 /opt

# Download rosserial library for Arduino
echo "Downloading rosserial library for Arduino..."
/opt/arduino-1.8.15/arduino --install-library rosserial_arduino

echo "Installation of ROS melodic, Freenect, TurtleBot packages, Arduino, and rosserial complete."
