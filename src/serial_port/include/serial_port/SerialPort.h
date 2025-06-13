#ifndef SERIALPORT_H
#define SERIALPORT_H
#include <vector>
#include <sstream>
#include <iomanip>
#include <serial/serial.h>
#include <iostream>
#include <ros/ros.h>
class SerialPort
{
public:
    // 由于uint8_t是无符号8位整数取值范围为0-255,pow(2,8) = 256
    // 常见波特率有：9600, 19200, 38400, 57600, 115200
    SerialPort(const std::string& port, uint32_t baudrate);
    ~SerialPort();

    void openSerialPort();
    void closeSerialPort();
    bool isOpen() const;
    bool readFrame(std::vector<uint8_t>& frame);
    void writeFrame(std::vector<uint8_t>& frame);

private:
    std::string port_;
    uint32_t baudrate_;
    serial::Serial ser_;
    // buffer_用于解决缓存和协议处理,遇到分包、粘包、数据不完整等情况就会失败。
    std::vector<uint8_t> buffer_;
};
#endif