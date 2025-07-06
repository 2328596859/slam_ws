#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "open_msgs/Serialmsg.h"
class FlameSensorInterface {
public:
    FlameSensorInterface(ros::NodeHandle& nh);
    void flameSensorDataCallback(const open_msgs::Serialmsg::ConstPtr& msg);
private:
    ros::Subscriber serial_data_sub_;
    ros::Publisher flame_pub_;
};