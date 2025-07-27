#include "device_ros_interface/platform_interface.h"

PlatformInterface::PlatformInterface(ros::NodeHandle& nh, PlatformDriver& platform_driver)
    : platform_driver_(platform_driver)
{
    platform_data_sub = nh.subscribe("serial_data", 1, &PlatformInterface::platformDataCallback, this);
    platform_pub_ = nh.advertise<open_msgs::PlatformState>("platform/platform_data", 10);
    move_sub_ = nh.subscribe("platform/platform_control", 1, &PlatformInterface::platformCallback, this);
}

void PlatformInterface::platformDataCallback(const open_msgs::Serialmsg::ConstPtr& msg) {
    if (msg->cmd1 == 0x25 && msg->cmd2 == 0x21) {
        open_msgs::PlatformState platform_msg;
        platform_msg.horizontalServoMotorStatus = msg->data[1];
        // DATA[0]：表示水平舵机id地址
        // DATA[1]：表示水平舵机目前状态：0x00：无异常状态，0x01：欠压电压异常保护中
        // 0x02：过压电压异常保护中，0x03：，
        // DATA[2]：表示水平舵机编码器码盘高八位
        // DATA[3]：表示水平舵机编码器码盘低八位
        // DATA[4]：表示垂直舵机id地址
        // DATA[5]：表示垂直舵机目前状态：0x00：无异常状态，0x01：欠压电压异常保护中
        // 0x02：过压电压异常保护中，0x03：，
        // DATA[6]：表示垂直舵机编码器码盘高八位
        // DATA[7]：表示垂直舵机编码器码盘低八位
        platform_msg.horizontalServoEncoderEngravingDegree = static_cast<int32_t>(msg->data[2]) << 8 | static_cast<int32_t>(msg->data[3]);
        platform_msg.verticalServoMotorStatus = msg->data[5];
        platform_msg.verticalServoEncoderEngravingDegree = static_cast<int32_t>(msg->data[6]) << 8 | static_cast<int32_t>(msg->data[7]);
        platform_pub_.publish(platform_msg);
        ROS_INFO("Platform state: horizontal motor status: %d, vertical motor status: %d, horizontal encoder: %d, vertical encoder: %d",
                 platform_msg.horizontalServoMotorStatus,
                 platform_msg.verticalServoMotorStatus,
                 platform_msg.horizontalServoEncoderEngravingDegree,
                 platform_msg.verticalServoEncoderEngravingDegree); 
    }
}

void PlatformInterface::platformCallback(const open_msgs::PlatformControl::ConstPtr& msg) {
    if (msg->cmd == "up") {
        uint16_t value = static_cast<uint16_t>(std::round(msg->data));
        uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);
        uint8_t low  = static_cast<uint8_t>(value & 0xFF);
        platform_driver_.platform_up(high, low); 
    }else if (msg->cmd == "down") {
        uint16_t value = static_cast<uint16_t>(std::round(msg->data));
        uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);
        uint8_t low  = static_cast<uint8_t>(value & 0xFF);
        platform_driver_.platform_down(high, low);
    }else if (msg->cmd == "right") {
        uint16_t value = static_cast<uint16_t>(std::round(msg->data));
        uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);
        uint8_t low  = static_cast<uint8_t>(value & 0xFF);
        platform_driver_.platform_right(high, low);
    } else if (msg->cmd == "left") {
        uint16_t value = static_cast<uint16_t>(std::round(msg->data));
        uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);
        uint8_t low  = static_cast<uint8_t>(value & 0xFF);
        platform_driver_.platform_left(high, low);
    }
}