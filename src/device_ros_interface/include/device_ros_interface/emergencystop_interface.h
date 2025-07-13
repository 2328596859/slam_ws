#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "open_msgs/Serialmsg.h"

class EmergencyStopInterface {
public:
    EmergencyStopInterface(ros::NodeHandle& nh);
    void emergencystopDataCallback(const open_msgs::Serialmsg::ConstPtr& msg);
private:
    ros::Subscriber serial_data_sub_;
    ros::Publisher emergencystop_pub_;
};