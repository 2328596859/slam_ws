#include <std_msgs/String.h>
#include "serial/SerialPort.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node2");
    ros::NodeHandle nh;

    std::string port = "/dev/pts/5"; // 串口设备路径
    SerialPort serialPort(port, 115200);

    serialPort.openSerialPort();
    if (!serialPort.isOpen()) {
        ROS_ERROR("无法打开串口，退出节点.");
        return -1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::String>("serial_msg", 10);
    ros::Rate loop_rate(10);
    ROS_INFO("wait for Frame");
    while (ros::ok()) {
        std::vector<uint8_t> frame;
        if (serialPort.readFrame(frame)) {
            std_msgs::String msg;
            std::stringstream ss;
            for (auto byte : frame) {
                ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            msg.data = ss.str();
            pub.publish(msg);
            ROS_INFO("Published: %s", msg.data.c_str());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    serialPort.closeSerialPort();
    return 0;
}