#include <std_msgs/String.h>
#include "serial_port/SerialPort.h"
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

    std::string port = "/dev/ttyS3";
    SerialPort serialPort(port, 115200);

    serialPort.openSerialPort();
    if (!serialPort.isOpen()) {
        ROS_ERROR("无法打开串口，退出节点.");
        return -1;
    }
    
    std::string topic_name = "serial_data";
    int loop_rate_hz = 50;
    
    // 使用线程运行串口通信
    std::thread serial_thread([&serialPort, &nh, &topic_name, loop_rate_hz]() {
        serialPort.publishSerialmsgs(nh, topic_name, loop_rate_hz);
    });
    
    serial_thread.detach();
    
    ROS_INFO("串口通信线程已启动，主线程继续执行其他任务...");
    
    // 主线程保持运行
    ros::spin();
    
    return 0;
}