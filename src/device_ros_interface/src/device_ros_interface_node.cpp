#include <ros/ros.h>
#include <thread>
#include "serial_port/SerialPort.h"
#include "device_driver/liftDriver.h"
#include "device_driver/platformDriver.h"
#include "device_ros_interface/lift_interface.h"
#include "device_ros_interface/platform_interface.h"
#include "device_ros_interface/smokesensor_interface.h"
#include "device_ros_interface/flamesensor_interface.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "device_ros_interface_node");
    ros::NodeHandle nh;

    SerialPort serial("/dev/ttyS3", 115200);
    serial.openSerialPort();
    if (!serial.isOpen()) {
        ROS_ERROR("Failed to open serial port!");
        return -1;
    }
    // 启动串口数据读取线程
    std::string topic_name = "serial_data";
    int loop_rate_hz = 50;
    std::thread serial_thread([&serial, &nh, &topic_name, loop_rate_hz]() {
        serial.publishSerialmsgs(nh, topic_name, loop_rate_hz);
    });
    serial_thread.detach();
    ROS_INFO("serial data thread started successfully.");
    // 创建设备驱动实例
    LiftDriver lift(serial);
    PlatformDriver platform(serial);

    LiftInterface lift_interface(nh, lift);
    PlatformInterface platform_interface(nh, platform);
    SmokeSensorInterface smoke_sensor_interface(nh);
    FlameSensorInterface flame_sensor_interface(nh);
    ROS_INFO("Device ROS interface node started successfully.");
    ros::spin();
    return 0;
}