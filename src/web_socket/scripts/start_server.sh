#!/bin/bash

# 进入脚本所在目录
cd "$(dirname "$0")"

# 加载 ROS 环境变量
source /opt/ros/kinetic/setup.bash

# 激活虚拟环境
source /home/shh/robot_ws/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/shh/slam_ws/src/web_socket/src
export PYTHONPATH=$PYTHONPATH:/home/shh/slam_ws/install_isolated/lib/python2.7/dist-packages

export ROS_LOG_DIR=/home/shh/websocket_logs/$(date +%Y-%m-%d)
# 等待 ROS master 可用
until rostopic list >/dev/null 2>&1; do
  echo "等待 ROS master 启动..."
  sleep 2
done

# 运行主服务脚本
python server_node.py
