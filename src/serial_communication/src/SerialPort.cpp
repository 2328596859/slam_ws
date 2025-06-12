#include "serial/SerialPort.hpp"

// 串口设置1000毫秒的超时,避免在读取数据时阻塞过久
SerialPort::SerialPort(const std::string& port, uint32_t baudrate)
    : port_(port), baudrate_(baudrate), ser_(port, baudrate, serial::Timeout::simpleTimeout(1)){
}
// 析构函数在初始化对象释放时自动调用关闭串口连接，避免资源泄漏
SerialPort::~SerialPort() {
    closeSerialPort();
}

void SerialPort::openSerialPort() {
    if (!ser_.isOpen()) {
        try{
            ser_.setPort(port_);
            ser_.setBaudrate(baudrate_);
            ser_.setBytesize(serial::eightbits);
            ser_.setStopbits(serial::stopbits_one);
            ser_.setParity(serial::parity_none);
            ser_.open();
            std::cout << "open successful: " << port_ << std::endl;
        }
        catch (serial::IOException& e){
            // cout相当于INFO日志输出，cerr相当于ERROR日志输出
            std::cerr << "Cannot open serial port: " << e.what() << std::endl;
        }
    }
}

void SerialPort::closeSerialPort() {
    if (ser_.isOpen()) {
        ser_.close();
        std::cout << "Serial port close successful: " << port_ << std::endl;
    }
}

bool SerialPort::isOpen() const {
    return ser_.isOpen();
}

// 串口的数据读写操作可个性化定义
// 读写帧的协议需要参考通信文档
bool SerialPort::readFrame(std::vector<uint8_t>& frame) {
    if (!ser_.isOpen()) return false;
    // 读取所有可用数据到缓存
    size_t available = ser_.available();
    if (available > 0) {
        std::vector<uint8_t> temp(available);
        ser_.read(temp, available);
        buffer_.insert(buffer_.end(), temp.begin(), temp.end());
    }

    // 组包：查找帧头、判断长度、校验
    while (buffer_.size() >= 7) { 
        // 查找帧头
        if (buffer_[0] != 0xA5) {
            buffer_.erase(buffer_.begin());
            continue;
        }
        uint8_t datalen = buffer_[1];
        size_t frame_len = 2 + datalen + 1; // 帧头+长度+数据+校验
        if (buffer_.size() < frame_len) break;

        // 校验
        uint8_t checksum = 0;
        for (size_t i = 0; i < frame_len - 1; ++i) checksum ^= buffer_[i];
        if (checksum != buffer_[frame_len - 1]) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        // 提取完整帧
        frame.assign(buffer_.begin(), buffer_.begin() + frame_len);
        buffer_.erase(buffer_.begin(), buffer_.begin() + frame_len);
        return true;
    }
    return false;

}

void SerialPort::writeFrame(std::vector<uint8_t>& frame) {
    if (!ser_.isOpen()) return;
    ser_.write(frame);
}