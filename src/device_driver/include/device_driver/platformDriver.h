#pragma once
#include "serial_port/SerialPort.h"
#include <vector>

class PlatformDriver {
public:
    explicit PlatformDriver(SerialPort& serial) : serial_(serial) {}
    void platform_up(uint8_t data1,uint8_t data2);
    void platform_down(uint8_t data1,uint8_t data2);
    void platform_right(uint8_t data1,uint8_t data2);
    void platform_left(uint8_t data1,uint8_t data2);
private:
    SerialPort& serial_;
    std::vector<uint8_t> buildFrame(uint8_t data0,uint8_t data1,uint8_t data2);
};