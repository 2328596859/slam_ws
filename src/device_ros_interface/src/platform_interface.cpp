#include "device_ros_interface/platform_interface.h"

PlatformInterface::PlatformInterface(ros::NodeHandle& nh, PlatformDriver& platform_driver)
    : platform_driver_(platform_driver)
{
    move_sub_ = nh.subscribe("platform_move", 1, &PlatformInterface::moveCallback, this);
}

void PlatformInterface::moveCallback(const std_msgs::Int32::ConstPtr& msg) {
    // 假设 msg->data 表示移动到的平台编号
    platform_driver_.moveTo(msg->data);
}