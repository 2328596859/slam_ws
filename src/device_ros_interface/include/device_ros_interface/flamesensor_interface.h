#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "open_msgs/Serialmsg.h"
#include "device_driver/flamesensorDriver.h"
#include "open_msgs/FlamesensorControl.h"
class FlameSensorInterface {
public:
    FlameSensorInterface(ros::NodeHandle& nh,FlamesensorDriver& flamesensor);
    void flameSensorDataCallback(const open_msgs::Serialmsg::ConstPtr& msg);
    void controlCallback(const open_msgs::FlamesensorControl::ConstPtr& msg);
private:
    ros::Subscriber serial_data_sub_,control_sub_ ;
    ros::Publisher  flame_state_pub_;
    FlamesensorDriver& flamesensor_;
};