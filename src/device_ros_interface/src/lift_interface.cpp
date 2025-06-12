#include "device_ros_interface/lift_interface.h"
#include <std_msgs/Empty.h>

LiftInterface::LiftInterface(ros::NodeHandle& nh, LiftDriver& lift_driver)
    : lift_driver_(lift_driver)
{
    lift_up_sub_ = nh.subscribe("lift_up", 1, &LiftInterface::liftUpCallback, this);
    lift_down_sub_ = nh.subscribe("lift_down", 1, &LiftInterface::liftDownCallback, this);
}

void LiftInterface::liftUpCallback(const std_msgs::Empty::ConstPtr&) {
    lift_driver_.liftUp();
}

void LiftInterface::liftDownCallback(const std_msgs::Empty::ConstPtr&) {
    lift_driver_.liftDown();
}