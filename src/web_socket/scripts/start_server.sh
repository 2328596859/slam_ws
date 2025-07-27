#!/bin/bash

# 进入脚本所在目录
cd "$(dirname "$0")"

# 加载 ROS 环境
source /opt/ros/kinetic/setup.bash

# 激活虚拟环境
source /home/shh/robot_ws/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/shh/slam_ws/src/web_socket/src
export PYTHONPATH=$PYTHONPATH:/home/shh/slam_ws/install_isolated/lib/python2.7/dist-packages

# 设置 ROS 日志目录
export ROS_LOG_DIR=/home/data/websocket_logs/$(date +%Y-%m-%d)
mkdir -p "$ROS_LOG_DIR"

# 等待 ROS master 启动
echo "[INFO] 等待 ROS master 启动..."
until rostopic list >/dev/null 2>&1; do
  sleep 1
done
echo "[INFO] ROS master 已就绪"

# 等待 device_ros_interface_node 启动（最多等待 30 次，每次 1s）
echo "[INFO] 等待 device_ros_interface_node 启动..."
for i in {1..60}; do
    if rosnode list | grep -q "/device_ros_interface_node"; then
        echo "[INFO] device_ros_interface_node 已启动"
        break
    fi
    echo "[INFO] 第 $i 秒：device_ros_interface_node 尚未启动"
    sleep 1
done

# 再等几秒以确保初始化完成（可选）
sleep 3

# 最终执行主服务
echo "[INFO] 启动 websocket 服务 server_node.py"
exec python server_node.py
