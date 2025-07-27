#!/bin/bash

cd /home/shh/slam_ws/
source install_isolated/setup.bash

# 等待串口设备准备好
echo "等待串口设备..."
for i in {1..60}; do
    if [ -e /dev/ttyUSB0 ]; then
        echo "串口已就绪"
        break
    fi
    echo "等待 /dev/ttyUSB0... ($i)"
    sleep 1
done

roslaunch device_ros_interface start.launch
