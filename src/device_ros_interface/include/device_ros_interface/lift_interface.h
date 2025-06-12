#pragma once
#include <ros/ros.h>
#include "device_driver/liftDriver.h"

class LiftInterface {
public:
    LiftInterface(ros::NodeHandle& nh, LiftDriver& lift_driver);
private:
    void liftUpCallback(const std_msgs::Empty::ConstPtr& msg);
    void liftDownCallback(const std_msgs::Empty::ConstPtr& msg);
    ros::Subscriber lift_up_sub_;
    ros::Subscriber lift_down_sub_;
    LiftDriver& lift_driver_;
};