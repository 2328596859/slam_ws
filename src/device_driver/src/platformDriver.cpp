#include "device_driver/platformDriver.h"

std::vector<uint8_t> PlatformDriver::buildFrame(uint8_t data0,uint8_t data1, uint8_t data2) {
    std::vector<uint8_t> frame;
    frame.push_back(0xA5);       // 帧头
    frame.push_back(0x05);       // 长度
    frame.push_back(0x55);       // 命令字
    frame.push_back(0x21);       // 功能码
    frame.push_back(data0);      // 参数1
    frame.push_back(data1);      // 参数2
    frame.push_back(data2);      // 参数3
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size(); ++i) checksum ^= frame[i];
    frame.push_back(checksum);  // 校验和
    return frame;
}

void PlatformDriver::platform_up(uint8_t data1, uint8_t data2) {
    auto frame = buildFrame(0x02, data1, data2); 
    serial_.writeFrame(frame);
}

void PlatformDriver::platform_down(uint8_t data1,uint8_t data2) {
    auto frame = buildFrame(0x03, data1, data2);
    serial_.writeFrame(frame);
}
void PlatformDriver::platform_right(uint8_t data1,uint8_t data2) {
    auto frame = buildFrame(0x01, data1, data2); 
    serial_.writeFrame(frame);
}
void PlatformDriver::platform_left(uint8_t data1, uint8_t data2) {
    auto frame = buildFrame(0x00, data1, data2);
    serial_.writeFrame(frame);
}
