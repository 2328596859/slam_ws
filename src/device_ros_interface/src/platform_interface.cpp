#include "device_ros_interface/platform_interface.h"

PlatformInterface::PlatformInterface(ros::NodeHandle& nh, PlatformDriver& platform_driver)
    : platform_driver_(platform_driver)
{
    move_sub_ = nh.subscribe("platform_control", 1, &PlatformInterface::platformCallback, this);
}

void PlatformInterface::platformCallback(const open_msgs::PlatformControl::ConstPtr& msg) {
    if (msg->cmd == "up") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        platform_driver_.platform_up(data1);
    } else if (msg->cmd == "down") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        platform_driver_.platform_down(data1);
    } else if (msg->cmd == "right") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        platform_driver_.platform_right(data1);
    } else if (msg->cmd == "left") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        platform_driver_.platform_left(data1);
    }
}