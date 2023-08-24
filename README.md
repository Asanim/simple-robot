### Project Overview

A ROS 2-based robot designed for autonomous operation in a simulated environment. The robot is capable of autonomous navigation, mapping, waypoint selection, and path planning. It integrates a Kinect V1 camera for object detection and depth sensing, while also implementing SLAM and path control. The system is integrated with AWS services, such as MQTT communication, and video streaming through Amazon Kinesis Video Streams. This project is designed for a Raspberry Pi 4 running Ubuntu MATE, utilizing ROS 2. 

## System Requirements

- **Hardware**: Raspberry Pi 4
- **Operating System**: [Ubuntu 22.04 LTS ARM64 Jammy](https://ubuntu-mate.org/download/arm64/jammy/)

### Robot Capabilities

- **Autonomous Navigation**: The robot can autonomously roam a room using the Kinect V1 sensor to map the environment and avoid obstacles. The system uses the GMapping SLAM algorithm to generate a 2D map of the room.
  
- **Waypoint Navigation and Path Planning**: The robot is capable of selecting waypoints and planning paths within the room. It utilizes the ROS 2 navigation stack with global and local planners to compute velocity commands for movement along the planned path.

- **Object Detection**: The Kinect V1 camera is used with a YOLOv11 node to detect objects in the environment, identifying their proximity via depth data.

- **AWS Greengrass MQTT Communication**: The robot communicates with AWS Greengrass using MQTT. It sends data on path planning, location, and velocity, and it can receive control commands to move the robot.

- **Video Streaming**: The robot streams live video using Amazon Kinesis Video Streams, providing remote monitoring of its actions.

### Installation and Setup

The installation script provided sets up the latest LTS version of ROS 2 and necessary dependencies, including Kinect V1 drivers, the navigation stack for differential drive robots, 2D SLAM (GMapping), and path planning. The script also installs Gazebo for simulation, YOLOv11 for object detection, AWS Greengrass for MQTT communication, and Kinesis Video Streams for live video streaming.

```bash
./scripts/install-ros-iron.sh
```


### Launch File

The launch file configures the launch of the robot's navigation stack, including:

- The Kinect V1 driver for depth sensing.
- The navigation stack with differential drive robot configuration.
- The GMapping SLAM algorithm for 2D mapping.
- A YOLOv11 node to read Kinect V1 camera input and output bounding box predictions.
- Integration of odometry data for feeding into the navigation stack and SLAM.
- Option to launch Gazebo simulation with a URDF for a simple differential drive robot.

### URDF

The URDF defines a simple differential drive robot with integrated odometry and Kinect V1 depth camera. This robot is set within a simulated world using Gazebo, with options for the Distribution Center World and Factory World. The URDF provides the structure for the robot's simulation environment, including its sensors and movement capabilities.

### AWS Integration

- **MQTT Node**: The AWS Greengrass MQTT node sends data on path planning, robot location (x, y), and velocity (twist) to AWS. It also listens for control commands via MQTT topics to adjust the robot's movement.
- **YOLOv11 Node**: The YOLOv11 node processes Kinect V1 input to output bounding box predictions for detected objects.
- **Depth Map Node**: This node compares the object detection results from YOLOv11 with depth data from the Kinect V1 to determine the proximity of detected objects.
- **Kinesis Video Streams**: The robot streams live video footage through Amazon Kinesis Video Streams, allowing remote monitoring of its actions.


## Installation Notes

- Currently using `sudo apt install ros-iron-desktop`, but for development builds, `sudo apt install ros-iron-ros-base` should be used instead.
- This repository is fairly extensive and will require pruning for optimization.
- The development PC is running ROS 2 iron.

## Setting Up ROS 2 Environment

1. **Set up ROS 2 environment**:
   ```bash
   source /opt/ros/iron/setup.bash
   ```

2. **Run example ROS 2 commands**:

   **Talker (C++)**:
   ```bash
   ros2 run demo_nodes_cpp talker
   ```

   **Listener (Python)**:
   ```bash
   ros2 run demo_nodes_py listener
   ```

3. **More ROS 2 examples**:
   - Subscriber example:
     ```bash
     ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
     ```
   - Publisher example:
     ```bash
     ros2 run examples_rclcpp_minimal_publisher publisher_member_function
     ```

4. **Creating a new package**:
   - CMake-based package:
     ```bash
     ros2 pkg create --build-type ament_cmake fake_odom
     ```
   - Python-based package with node:
     ```bash
     ros2 pkg create --build-type ament_python --node-name odom_publisher fake_odom --license Apache-2.0
     ```
   - Running a package:
     ```bash
     conda deactivate
     source install/setup.bash
     ros2 run fake_odom odom_publisher --ros-args --log-level debug
     ```

## ROS 1 Modules (for legacy support)

### Freenect for Kinect Camera
- To use the Kinect V1 camera with ROS 1, the Freenect stack is required.
  - GitHub repository: [freenect_stack](https://github.com/ros-drivers/freenect_stack.git)

### rosserial for Arduino Mega
- The rosserial package allows communication with an Arduino device via ROS. Install rosserial and rosserial_arduino for Arduino Mega interaction.
  - GitHub repository: [rosserial](https://github.com/ros-drivers/rosserial)
  - ROS Wiki: [rosserial](http://wiki.ros.org/rosserial)
  - Arduino Library: [rosserial-arduino-library](https://www.arduino.cc/reference/en/libraries/rosserial-arduino-library/)

- Install `rosserial` and `rosserial_arduino` for ROS 1:
  ```bash
  sudo apt-get install ros-iron-rosserial-arduino
  sudo apt-get install ros-iron-rosserial
  ```

### Setting up Freenect and rosserial for ROS 1

1. **Install dependencies**:
   ```bash
   sudo apt-get update
   sudo apt-get install libapr1
   sudo apt-get install uuid-dev
   ```

2. **Clone repositories**:
   ```bash
   git clone https://github.com/ros-drivers/rosserial.git
   git clone https://github.com/ros-drivers/freenect_stack.git
   ```



   ros2 pkg create --build-type ament_cmake cpp_pubsub
   ros2 pkg create --build-type ament_python --node-name yolov7 simple_inference

   rosdep install -i --from-path src --rosdistro rolling -y



   ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"