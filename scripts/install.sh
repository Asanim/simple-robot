#!/bin/bash
set -e

# Update and upgrade
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y software-properties-common curl wget git python3-colcon-common-extensions python3-rosdep

# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release

if [ ! -d /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
fi

# Install latest LTS version of ROS 2 (jazzy or Rolling)
sudo apt update && sudo apt install -y ros-jazzy-desktop

# Source ROS 2
echo "source /opt/ros/jazzy/setup.bash" >>~/.bashrc
source ~/.bashrc

# Install ROS 2 dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Kinect Freenect and ROS 2 Wrapper
sudo apt install -y ros-jazzy-image-pipeline ros-jazzy-vision-msgs ros-jazzy-perception-pcl
mkdir -p ~/simple-robot/src && cd ~/simple-robot/src
git clone https://github.com/fadlio/kinect_ros2.git
cd ~/simple-robot && colcon build --symlink-install

# Install Navigation Stack for Differential Drive Robots
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox

# Install GMapping for 2D SLAM
sudo apt install -y ros-jazzy-slam-gmapping

# Install Gazebo and Simulation Tools
sudo apt install -y gazebo11 ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-plugins

# Install dependencies for YOLOv4
sudo apt-get update
sudo apt-get install ros-jazzy-image-transport ros-jazzy-cv-bridge
sudo apt-get install libopencv-dev python3-opencv
pip install torch torchvision opencv-python numpy

# Install Greengrass SDK
pip install greengrasssdk

# Install other necessary dependencies
pip install matplotlib  # For visualization purposes if needed

# Install YOLOv11 (assuming a compatible version is available)
mkdir -p ~/simple-robot/src && cd ~/simple-robot/src
git clone https://github.com/ultralytics/yolov11.git
cd yolov11 && pip install -r requirements.txt

# Install AWS Greengrass and MQTT Client
sudo apt install -y ros-jazzy-aws-iot-ros2

# Install Amazon Kinesis Video Streams SDK
mkdir -p ~/simple-robot/src && cd ~/simple-robot/src
git clone https://github.com/aws-robotics/kinesisvideo-ros2.git
cd ~/simple-robot && colcon build --symlink-install

# Source the workspace
echo "source ~/simple-robot/install/setup.bash" >>~/.bashrc
source ~/.bashrc

# Create a systemd service to launch bringup node on startup
cat <<EOF | sudo tee /etc/systemd/system/ros2_bringup.service
[Unit]
Description=ROS 2 Bringup
After=network.target

[Service]
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && ros2 launch my_robot_bringup bringup.launch.py'
Restart=always
User=$USER

[Install]
WantedBy=multi-user.target
EOF

# Enable the service
sudo systemctl daemon-reload
sudo systemctl enable ros2_bringup.service

echo "ROS 2 setup complete. Please reboot your system."
