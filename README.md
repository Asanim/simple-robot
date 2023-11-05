# Simple Robot
Raspberry pi 4 with ubuntu mate installed
* [ubuntu 22.04 lts arm64 jammy](https://ubuntu-mate.org/download/arm64/jammy/)

Currently using [ros2 iron](https://docs.ros.org/en/iron/Installation.html) which is supported until 2024. To see an updated [list of distros](https://docs.ros.org/en/rolling/Releases.html)


## Notes
* currently using sudo apt install ros-iron-desktop when should be sudo apt install ros-iron-ros-base for development build
* this repo is fairly expansive and will require pruning 
* the dev pc uses ros2 galactic 


## ROS examples

source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_cpp talker

source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_py listener

[ros beginner tutorials](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)


ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
ros2 run examples_rclcpp_minimal_publisher publisher_member_function

creating a new package

ros2 pkg create --build-type ament_cmake fake_odom

ros2 pkg create --build-type ament_python --node-name odom_publisher fake_odom --license Apache-2.0

conda deactivate
source install/setup.bash
ros2 run fake_odom odom_publisher --ros-args --log-level debug

# ROS 1 modules
* freenect for kinect camera
* rosserial for mega
https://github.com/ros-drivers/freenect_stack.git


## ROS 1 bridge
https://docs.ros.org/en/rolling/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html



## Inital commits using ROS 1 not ROS2 


### Rosserial
as most of the 


https://github.com/ros-drivers/rosserial
http://wiki.ros.org/rosserial
https://www.arduino.cc/reference/en/libraries/rosserial-arduino-library/


http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial


http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup


### Freenect stack

https://github.com/ros-drivers/freenect_stack


### init build 

sudo apt-get update
sudo apt-get install libapr1
sudo apt-get install uuid-dev

git clone https://github.com/ros-drivers/rosserial.git
git clone https://github.com/ros-drivers/freenect_stack.git



# AWS robomaker

https://aws.amazon.com/blogs/iot/orchestrate-nvidia-isaac-sim-ros-2-navigation-aws-robomaker-public-container-image/

