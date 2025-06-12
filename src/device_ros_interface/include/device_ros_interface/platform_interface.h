#pragma once
#include <ros/ros.h>
#include "device_driver/platformDriver.h"
#include <std_msgs/Int32.h>

class PlatformInterface {
public:
    PlatformInterface(ros::NodeHandle& nh, PlatformDriver& platform_driver);
private:
    void moveCallback(const std_msgs::Int32::ConstPtr& msg);
    ros::Subscriber move_sub_;
    PlatformDriver& platform_driver_;
};