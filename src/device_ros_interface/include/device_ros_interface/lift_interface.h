#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "device_driver/liftDriver.h"
#include "open_msgs/LiftControl.h"

class LiftInterface {
public:
    LiftInterface(ros::NodeHandle& nh, LiftDriver& lift_driver);
private:
    void controlCallback(const open_msgs::LiftControl::ConstPtr& msg);
    void liftDataCallback(const open_msgs::Serialmsg::ConstPtr& msg);
    ros::Subscriber control_sub_;
    ros::Subscriber lift_data_sub_;
    ros::Publisher lift_pub_;
    LiftDriver& lift_driver_;
};