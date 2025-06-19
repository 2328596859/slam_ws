#pragma once
#include <ros/ros.h>
#include "device_driver/platformDriver.h"
#include "open_msgs/PlatformControl.h"

class PlatformInterface {
public:
    PlatformInterface(ros::NodeHandle& nh, PlatformDriver& platform_driver);
private:
    void platformCallback(const open_msgs::PlatformControl::ConstPtr& msg);
    ros::Subscriber move_sub_;
    PlatformDriver& platform_driver_;
};