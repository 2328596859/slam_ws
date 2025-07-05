#include <ros/ros.h>
#include "serial_port/SerialPort.h"
#include "device_driver/liftDriver.h"
#include "device_driver/platformDriver.h"
#include "device_ros_interface/lift_interface.h"
#include "device_ros_interface/platform_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "device_ros_interface_node");
    ros::NodeHandle nh;

    SerialPort serial("/dev/ttyS3", 115200);
    serial.openSerialPort();
    if (!serial.isOpen()) {
        ROS_ERROR("Failed to open serial port!");
        return -1;
    }
    LiftDriver lift(serial);
    PlatformDriver platform(serial);

    LiftInterface lift_interface(nh, lift);
    PlatformInterface platform_interface(nh, platform);
    ROS_INFO("Device ROS interface node started successfully.");
    ros::spin();
    return 0;
}