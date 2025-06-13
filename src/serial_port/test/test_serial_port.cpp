#include <gtest/gtest.h>
#include "serial_port/SerialPort.h"

TEST(SerialPortTest, OpenClose) {
    SerialPort port("/dev/null", 9600);
    EXPECT_NO_THROW(port.openSerialPort());
    port.closeSerialPort();
    EXPECT_FALSE(port.isOpen());
}