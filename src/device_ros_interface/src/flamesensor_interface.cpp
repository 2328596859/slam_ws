#include "device_ros_interface/flamesensor_interface.h"  
FlameSensorInterface::FlameSensorInterface(ros::NodeHandle& nh,FlamesensorDriver& flamesensor) 
    :flamesensor_(flamesensor)
{
    serial_data_sub_ = nh.subscribe("serial_data", 1, &FlameSensorInterface::flameSensorDataCallback, this);
    flame_state_pub_ = nh.advertise<std_msgs::Int32>("flamesensor/flame_state", 10);
    control_sub_ = nh.subscribe("flamesensor/flamesensor_control", 1, &FlameSensorInterface::controlCallback, this);
}
void FlameSensorInterface::flameSensorDataCallback(const open_msgs::Serialmsg::ConstPtr& msg) {
    if (msg->cmd1 ==0x25  && msg->cmd2 == 0x01 ) {
        std_msgs::Int32 flame_msg;
        flame_msg.data = static_cast<int32_t>(msg->data[0]);
        flame_state_pub_.publish(flame_msg);
    }
}

void FlameSensorInterface::controlCallback(const open_msgs::FlamesensorControl::ConstPtr& msg) {
    if (msg->cmd == "cancle") {
        uint8_t data0 = static_cast<uint8_t>(msg->data);
        flamesensor_.cancle_alarm(data0);
    }
}