#include "device_driver/platformDriver.h"

std::vector<uint8_t> PlatformDriver::buildFrame(uint8_t func, uint8_t param1, uint8_t param2) {
    std::vector<uint8_t> frame;
    frame.push_back(0xA5);      // 帧头
    frame.push_back(0x04);      // 长度
    frame.push_back(0x55);      // 命令字
    frame.push_back(func);      // 功能码
    frame.push_back(param1);    // 参数1
    frame.push_back(param2);    // 参数2
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size(); ++i) checksum ^= frame[i];
    frame.push_back(checksum);  // 校验和
    return frame;
}

void PlatformDriver::platform_up() {
    auto frame = buildFrame(0x20, 0x00, 0x01); // 功能码0x20, 参数0x00, 0x01
    serial_.writeFrame(frame);
}

void PlatformDriver::platform_down() {
    auto frame = buildFrame(0x20, 0x01, 0x01); // 功能码0x20, 参数0x01, 0x01
    serial_.writeFrame(frame);
}
void PlatformDriver::platform_right() {
    auto frame = buildFrame(0x20, 0x02, 0x01); // 功能码0x20, 参数0x02, 0x01
    serial_.writeFrame(frame);
}
void PlatformDriver::platform_left() {
    auto frame = buildFrame(0x20, 0x03, 0x01); // 功能码0x20, 参数0x03, 0x01
    serial_.writeFrame(frame);
}
// End of src/device_driver/src/platformDriver.cpp