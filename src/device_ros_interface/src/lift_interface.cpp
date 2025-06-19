#include "device_ros_interface/lift_interface.h"

LiftInterface::LiftInterface(ros::NodeHandle& nh, LiftDriver& lift_driver)
    : lift_driver_(lift_driver)
{
    control_sub_ = nh.subscribe("lift_control", 1, &LiftInterface::controlCallback, this);
}

void LiftInterface::controlCallback(const open_msgs::LiftControl::ConstPtr& msg) {
    if (msg->cmd == "up") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        lift_driver_.liftUp(data1);
    } else if (msg->cmd == "down") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        lift_driver_.liftDown(data1);
    }
}