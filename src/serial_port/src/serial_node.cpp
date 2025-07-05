#include <std_msgs/String.h>
#include "serial_port/SerialPort.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node2");
    ros::NodeHandle nh;

    std::string port = "/dev/ttyS3"; // 串口设备路径
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
            if (frame.size() >= 7) { 
                uint8_t frame_head = frame[0];
                uint8_t data_len   = frame[1];
                uint8_t cmd        = frame[2];
                uint8_t func       = frame[3];
                uint8_t param1     = frame[4];
                uint8_t param2     = frame[5];
                uint8_t checksum   = frame[6];

                // 你可以根据需要打印或使用这些变量
                ROS_INFO("帧头: 0x%02x, 长度: %d, 命令: 0x%02x, 功能: 0x%02x, 参数1: 0x%02x, 参数2: 0x%02x, 校验: 0x%02x",
                        frame_head, data_len, cmd, func, param1, param2, checksum);
            }
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