#pragma once
#include "serial_port/serial/SerialPort.h"
#include <vector>

class LiftDriver {
public:
    explicit LiftDriver(SerialPort& serial) : serial_(serial) {}

    // 升降杆上升
    void liftUp();
    // 升降杆下降
    void liftDown();

private:
    SerialPort& serial_;
    std::vector<uint8_t> buildFrame(uint8_t func, uint8_t param1, uint8_t param2);
};