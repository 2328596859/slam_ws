#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "open_msgs/Serialmsg.h"
#include "open_msgs/SmokesensorControl.h"
#include "device_driver/smokesensorDriver.h"
class SmokeSensorInterface {
public:
    SmokeSensorInterface(ros::NodeHandle& nh,SmokesensorDriver& smokesensor);
    void smokesensorDataCallback(const open_msgs::Serialmsg::ConstPtr& msg);
    void controlCallback(const open_msgs::SmokesensorControl::ConstPtr& msg);
private:
    ros::Subscriber serial_data_sub_,control_sub_;
    ros::Publisher smoke_pub_,smoke_state_pub_;
    ros::Publisher temperature_pub_;
    ros::Publisher humidity_pub_;
    SmokesensorDriver& smokesensor_;
};