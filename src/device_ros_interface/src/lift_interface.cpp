#include "device_ros_interface/lift_interface.h"

LiftInterface::LiftInterface(ros::NodeHandle& nh, LiftDriver& lift_driver)
    : lift_driver_(lift_driver)
{
    control_sub_ = nh.subscribe("lift/lift_control", 1, &LiftInterface::controlCallback, this);
    lift_data_sub_ = nh.subscribe("serial_data", 1, &LiftInterface::liftDataCallback, this);
    lift_pub_ = nh.advertise<std_msgs::Int32>("lift/lift_data", 10);
}

void LiftInterface::controlCallback(const open_msgs::LiftControl::ConstPtr& msg) {
    if (msg->cmd == "up") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        lift_driver_.liftUp(data1);
    } else if (msg->cmd == "down") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        lift_driver_.liftDown(data1);
    }else if (msg->cmd == "reset") {
        uint8_t data1 = static_cast<uint8_t>(msg->data);
        lift_driver_.liftreset(data1);
    }
}

void LiftInterface::liftDataCallback(const open_msgs::Serialmsg::ConstPtr& msg) {
    if (msg->cmd1 == 0x25 && msg->cmd2 == 0x04) {
        std_msgs::Int32 lift_msg;
        lift_msg.data = static_cast<int32_t>(msg->data[0]);
        lift_pub_.publish(lift_msg);
    }
}