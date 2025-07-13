#include "device_ros_interface/emergencystop_interface.h"  
EmergencyStopInterface::EmergencyStopInterface(ros::NodeHandle& nh) {
    serial_data_sub_ = nh.subscribe("serial_data", 1, &EmergencyStopInterface::emergencystopDataCallback, this);
    emergencystop_pub_ = nh.advertise<std_msgs::Int32>("emergencystop/emergency_data", 10);
}
void EmergencyStopInterface::emergencystopDataCallback(const open_msgs::Serialmsg::ConstPtr& msg) {
    if (msg->cmd1 == 0x25 && msg->cmd2 == 0x03) {
        std_msgs::Int32 emergency_msg;
        emergency_msg.data = static_cast<int32_t>(msg->data[0]);
        emergencystop_pub_.publish(emergency_msg);
    }
}