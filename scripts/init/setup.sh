#!/bin/bash

# /home/robot/simple-robot/scripts

# sudo apt update && sudo apt dist-upgrade -y
sudo apt install -y git openssh-server python3 python3-pip vim ros-rolling-urdf-tutorial ros-rolling-rosbag2

cd ~/
git clone https://github.com/ibaiGorordo/ONNX-YOLOv7-Object-Detection.git 
wget https://nvidia.box.com/shared/static/mvdcltm9ewdy2d5nurkiqorofz1s53ww.whl -O onnxruntime_gpu-1.15.1-cp38-cp38m-linux_aarch64.whl
pip3 install onnxruntime_gpu-1.15.1-cp38-cp38m-linux_aarch64.whl

pip3 install -r ~/simple-robot/scripts/requirements.txt

export PATH=/home/robot/.local/bin:$PATH