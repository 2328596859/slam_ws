#pragma once
#include "serial_port/SerialPort.h"
#include <vector>

class LiftDriver {
public:
    explicit LiftDriver(SerialPort& serial) : serial_(serial) {}
    void liftUp(uint8_t data1);
    void liftDown(uint8_t data1);

private:
    SerialPort& serial_;
    std::vector<uint8_t> buildFrame(uint8_t data0,uint8_t data1);
};