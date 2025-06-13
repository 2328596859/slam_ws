#include "device_driver/liftDriver.h"

// 构造帧的数据CMD1和CMD2都已经确定只需传入data即可
std::vector<uint8_t> LiftDriver::buildFrame(uint8_t data0,uint8_t data1) {
    std::vector<uint8_t> frame;
    frame.push_back(0xA5);       // 帧头
    frame.push_back(0x04);       // 长度
    frame.push_back(0x55);       // CMD1
    frame.push_back(0x20);       // CMD2
    frame.push_back(data0);      // data0
    frame.push_back(data1);      // data1
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size(); ++i) checksum ^= frame[i];
    frame.push_back(checksum);  // 校验和
    return frame;
}

void LiftDriver::liftUp(uint8_t data1) {
    auto frame = buildFrame(0x00,data1);
    serial_.writeFrame(frame);
}

void LiftDriver::liftDown(uint8_t data1) {
    auto frame = buildFrame(0x01,data1);
    serial_.writeFrame(frame);
}