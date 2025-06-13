#include "serial_port/SerialPort.h"
#include "device_driver/liftDriver.h"
#include "device_driver/platformDriver.h"

int main() {
    SerialPort serial("/dev/ttyUSB0", 115200);
    serial.openSerialPort();
    if (!serial.isOpen()) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return -1;
    }
    LiftDriver lift(serial);
    PlatformDriver platform(serial);
    // // 控制升降机
    // lift.liftUp();
    // lift.liftDown();
    // // 控制平台
    // platform.platform_up();
    // platform.platform_down();
    // platform.platform_right();
    // platform.platform_left();   
}