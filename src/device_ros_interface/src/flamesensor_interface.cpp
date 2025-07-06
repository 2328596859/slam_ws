#include "device_ros_interface/flamesensor_interface.h"  
FlameSensorInterface::FlameSensorInterface(ros::NodeHandle& nh) {
    serial_data_sub_ = nh.subscribe("serial_data", 1, &FlameSensorInterface::flameSensorDataCallback, this);
    flame_pub_ = nh.advertise<std_msgs::Int32>("flamesensor/flame_data", 10);
}
void FlameSensorInterface::flameSensorDataCallback(const open_msgs::Serialmsg::ConstPtr& msg) {
    if (msg->cmd2 != 1) {
        return;
    }
    std_msgs::Int32 flame_msg;
    flame_msg.data = static_cast<int32_t>(msg->data[0]);
    flame_pub_.publish(flame_msg);
}