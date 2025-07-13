#pragma once
#include "serial_port/SerialPort.h"
#include <vector>

class SmokesensorDriver {
public:
    explicit SmokesensorDriver(SerialPort& serial) : serial_(serial) {}
    void cancle_alarm(uint8_t data0);

private:
    SerialPort& serial_;
    std::vector<uint8_t> buildFrame(uint8_t data0);
};