#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <vector>
#include <sstream>
#include <iomanip>

class SerialCommunication
{
public:
    SerialCommunication(const std::string& port, uint32_t baudrate = 9600)
    {
        port_ = port;
        baudrate_ = baudrate;
        openSerialPort();
    }

    ~SerialCommunication()
    {
        closeSerialPort();
    }

    void openSerialPort()
    {
        try
        {
            ser_.setPort(port_);
            ser_.setBaudrate(baudrate_);
            ser_.setBytesize(serial::eightbits);
            ser_.setStopbits(serial::stopbits_one);
            ser_.setParity(serial::parity_none);
            ser_.open();
            ROS_INFO("串口打开成功: %s", port_.c_str());
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("无法打开串口: %s", e.what());
        }
    }

    void closeSerialPort()
    {
        if (ser_.isOpen())
        {
            ser_.close();
            ROS_INFO("串口已关闭: %s", port_.c_str());
        }
    }

    bool isOpen() const
    {
        return ser_.isOpen();
    }

    // 读取并组包
    bool readFrame(uint8_t& cmd, uint8_t& data)
    {
        static std::vector<uint8_t> buffer;
        size_t available = ser_.available();
        if (available > 0)
        {
            std::vector<uint8_t> temp(available);
            ser_.read(temp, available);
            buffer.insert(buffer.end(), temp.begin(), temp.end());
        }

        // 查找帧头
        while (buffer.size() >= 6)
        {
            if (buffer[0] != 0xA5)
            {
                buffer.erase(buffer.begin());
                continue;
            }
            // 帧长
            uint16_t datalen = (buffer[1] << 8) | buffer[2];
            if (datalen != 2)
            {
                buffer.erase(buffer.begin());
                continue;
            }
            // 检查长度
            if (buffer.size() < 6)
                return false;
            // 校验
            uint8_t checksum = 0;
            for (int i = 0; i < 5; ++i)
                checksum ^= buffer[i];
            if (checksum != buffer[5])
            {
                buffer.erase(buffer.begin());
                continue;
            }
            // 提取数据
            cmd = buffer[3];
            data = buffer[4];
            buffer.erase(buffer.begin(), buffer.begin() + 6);
            return true;
        }
        return false;
    }

    // 发送一帧
    void writeFrame(uint8_t cmd, uint8_t data)
    {
        std::vector<uint8_t> frame;
        frame.push_back(0xA5);         // 帧头
        frame.push_back(0x00);         // datalen高字节
        frame.push_back(0x02);         // datalen低字节（固定2）
        frame.push_back(cmd);          // 指令
        frame.push_back(data);         // 数据
        uint8_t checksum = 0;
        for (int i = 0; i < 5; ++i)
            checksum ^= frame[i];
        frame.push_back(checksum);     // 校验
        ser_.write(frame);
        ROS_INFO("已发送帧: cmd=0x%02X, data=0x%02X", cmd, data);
    }

private:
    std::string port_;
    uint32_t baudrate_;
    serial::Serial ser_;
};

// ROS节点功能
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_communication_node");
    ros::NodeHandle nh;

    std::string port = "/dev/ttyUSB0";
    SerialCommunication serialComm(port);

    if (!serialComm.isOpen())
    {
        ROS_ERROR("无法打开串口，退出节点.");
        return -1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::String>("serial_msg", 10);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        uint8_t cmd = 0, data = 0;
        while (serialComm.readFrame(cmd, data))
        {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "cmd=" << std::hex << std::setw(2) << std::setfill('0') << (int)cmd
               << ", data=" << std::hex << std::setw(2) << std::setfill('0') << (int)data;
            msg.data = ss.str();
            pub.publish(msg);
            ROS_INFO("收到帧: %s", msg.data.c_str());
        }
        // serialComm.writeFrame(0x01, 0x02);
        ros::spinOnce();
        loop_rate.sleep();
    }

    serialComm.closeSerialPort();
    return 0;
}