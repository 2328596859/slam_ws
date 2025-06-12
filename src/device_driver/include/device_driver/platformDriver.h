#pragma once
#include "serial_port/serial/SerialPort.h"
#include <vector>

class PlatformDriver {
public:
    explicit PlatformDriver(SerialPort& serial) : serial_(serial) {}
    void platform_up();
    void platform_down();
    void platform_right();
    void platform_left();
private:
    SerialPort& serial_;
    std::vector<uint8_t> buildFrame(uint8_t func, uint8_t param1, uint8_t param2);
};