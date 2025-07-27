#pragma once
#include <ros/ros.h>
#include "device_driver/platformDriver.h"
#include "open_msgs/PlatformControl.h"
#include "open_msgs/PlatformState.h"

class PlatformInterface {
public:
    PlatformInterface(ros::NodeHandle& nh, PlatformDriver& platform_driver);
    void platformDataCallback(const open_msgs::Serialmsg::ConstPtr& msg);
private:
    void platformCallback(const open_msgs::PlatformControl::ConstPtr& msg);
    ros::Subscriber move_sub_;
    ros::Subscriber platform_data_sub;
    ros::Publisher platform_pub_;
    PlatformDriver& platform_driver_;
};