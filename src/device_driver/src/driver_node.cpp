#include "serial_port/serial/SerialPort.h"
#include "device_driver/liftDriver.h"

int main() {
    SerialPort serial("/dev/ttyUSB0", 115200);
    serial.openSerialPort();
    LiftDriver lift(serial);

    // 控制升降杆上升
    lift.liftUp();

    // 控制升降杆下降
    lift.liftDown();

    serial.closeSerialPort();
    return 0;
}