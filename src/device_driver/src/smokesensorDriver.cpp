#include "device_driver/smokesensorDriver.h"

// 构造帧的数据CMD1和CMD2都已经确定只需传入data即可
std::vector<uint8_t> SmokesensorDriver::buildFrame(uint8_t data0) {
    std::vector<uint8_t> frame;
    frame.push_back(0xA5);       // 帧头
    frame.push_back(0x03);       // 长度
    frame.push_back(0x55);       // CMD1
    frame.push_back(0x23);       // CMD2
    frame.push_back(data0);      // data0
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size(); ++i) checksum ^= frame[i];
    frame.push_back(checksum);  // 校验和
    return frame;
}

void SmokesensorDriver::cancle_alarm(uint8_t data0) {
    auto frame = buildFrame(0x01);
    serial_.writeFrame(frame);
}
