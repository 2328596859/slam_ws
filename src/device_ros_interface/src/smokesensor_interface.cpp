#include "device_ros_interface/smokesensor_interface.h"
#include <std_msgs/Float64.h>
#include <cmath>
#include <std_msgs/Int32.h>
SmokeSensorInterface::SmokeSensorInterface(ros::NodeHandle& nh,SmokesensorDriver& smokesensor) 
    :smokesensor_(smokesensor)
{
    serial_data_sub_ = nh.subscribe("serial_data", 1, &SmokeSensorInterface::smokesensorDataCallback, this);
    smoke_pub_ = nh.advertise<std_msgs::Float64>("smokesensor/smoke_data", 10);
    smoke_state_pub_ = nh.advertise<std_msgs::Int32>("smokesensor/smoke_state", 10);
    temperature_pub_ = nh.advertise<std_msgs::Float64>("smokesensor/temperature_data", 10);
    humidity_pub_ = nh.advertise<std_msgs::Float64>("smokesensor/humidity_data", 10);
    control_sub_ = nh.subscribe("smokesensor/smokesensor_control", 1, &SmokeSensorInterface::controlCallback, this);
}
void SmokeSensorInterface::smokesensorDataCallback(const open_msgs::Serialmsg::ConstPtr& msg) {
    if (msg->cmd2 == 0 && msg->cmd1 == 0x25) {
        std_msgs::Float64 smoke_msg;
        std_msgs::Float64 temperature_msg;
        std_msgs::Float64 humidity_msg;
        // 开始处理数据
        // 处理msg->data中的6个字节数据前2个字节为烟雾数据，中间2个字节为温度,最后2个字节为湿度数据
        // 高八位在前，低八位在后,先复原数据为16进制后重新组合高八位和低八位
        
        // 烟雾数据 (字节0-1): 将十进制还原为16进制，然后组合
        uint8_t smoke_high = static_cast<uint8_t>(msg->data[0]);  // 高八位
        uint8_t smoke_low = static_cast<uint8_t>(msg->data[1]);   // 低八位
        uint16_t smoke_raw = (smoke_high << 8) | smoke_low;
        smoke_msg.data = static_cast<double>(smoke_raw);
        
        // 温度数据 (字节2-3): 将十进制还原为16进制，然后组合
        uint8_t temp_high = static_cast<uint8_t>(msg->data[2]);   // 高八位
        uint8_t temp_low = static_cast<uint8_t>(msg->data[3]);    // 低八位
        uint16_t temp_raw = (temp_high << 8) | temp_low;
        double temp_value = static_cast<double>(temp_raw) / 100.0;  
        temperature_msg.data = round(temp_value * 100.0) / 100.0;  // 保留2位小数
        
        // 湿度数据 (字节4-5): 将十进制还原为16进制，然后组合
        uint8_t humid_high = static_cast<uint8_t>(msg->data[4]);  // 高八位
        uint8_t humid_low = static_cast<uint8_t>(msg->data[5]);   // 低八位
        uint16_t humid_raw = (humid_high << 8) | humid_low;
        double humid_value = static_cast<double>(humid_raw) / 100.0;
        humidity_msg.data = round(humid_value * 100.0) / 100.0;  // 保留2位小数

        smoke_pub_.publish(smoke_msg);
        temperature_pub_.publish(temperature_msg);
        humidity_pub_.publish(humidity_msg);
    }
    else if (msg->cmd1 == 0x25 && msg->cmd2 == 0x02)
    {
        std_msgs::Int32 smoke_state_msg;
        smoke_state_msg.data = static_cast<int32_t>(msg->data[0]);
        smoke_state_pub_.publish(smoke_state_msg);
    }
    
}

void SmokeSensorInterface::controlCallback(const open_msgs::SmokesensorControl::ConstPtr& msg) {
    if (msg->cmd == "cancle") {
        uint8_t data0 = static_cast<uint8_t>(msg->data);
        smokesensor_.cancle_alarm(data0);
    }
}