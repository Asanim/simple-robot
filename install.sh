#!/bin/bash
set -e

# Update and upgrade
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y software-properties-common curl wget git python3-colcon-common-extensions python3-rosdep

# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install latest LTS version of ROS 2 (Humble or Rolling)
sudo apt update && sudo apt install -y ros-humble-desktop

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ROS 2 dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Kinect Freenect and ROS 2 Wrapper
sudo apt install -y ros-humble-image-pipeline ros-humble-vision-msgs ros-humble-perception-pcl
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/fadlio/kinect_ros2.git
cd ~/ros2_ws && colcon build --symlink-install

# Install Navigation Stack for Differential Drive Robots
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# Install GMapping for 2D SLAM
sudo apt install -y ros-humble-slam-gmapping

# Install Gazebo and Simulation Tools
sudo apt install -y gazebo11 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Install YOLOv11 (assuming a compatible version is available)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ultralytics/yolov11.git
cd yolov11 && pip install -r requirements.txt

# Install AWS Greengrass and MQTT Client
sudo apt install -y ros-humble-aws-iot-ros2

# Install Amazon Kinesis Video Streams SDK
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/aws-robotics/kinesisvideo-ros2.git
cd ~/ros2_ws && colcon build --symlink-install

# Source the workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create a systemd service to launch bringup node on startup
cat <<EOF | sudo tee /etc/systemd/system/ros2_bringup.service
[Unit]
Description=ROS 2 Bringup
After=network.target

[Service]
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 launch my_robot_bringup bringup.launch.py'
Restart=always
User=$USER

[Install]
WantedBy=multi-user.target
EOF

# Enable the service
sudo systemctl daemon-reload
sudo systemctl enable ros2_bringup.service


echo "ROS 2 setup complete. Please reboot your system."
