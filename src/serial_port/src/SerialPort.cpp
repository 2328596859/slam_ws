#include "serial_port/SerialPort.h"

// 串口设置1000毫秒的超时,避免在读取数据时阻塞过久
SerialPort::SerialPort(const std::string& port, uint32_t baudrate)
    : port_(port), baudrate_(baudrate), ser_(port, baudrate, serial::Timeout::simpleTimeout(100)){
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
        
        // 输出buffer
        std::ostringstream oss;
        for (const auto& byte : buffer_) {
            oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        // ROS_INFO_STREAM("Buffer content: " << oss.str());
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
    ROS_INFO("Write frame successful");
    // 输出写入的帧数据
    std::ostringstream oss;
    for (const auto& byte : frame) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    ROS_INFO_STREAM("Wrote frame: " << oss.str());
}

void SerialPort::publishSerialmsgs(ros::NodeHandle& nh, const std::string& topic_name, int loop_rate_hz) {
    if (!ser_.isOpen()) {
        ROS_ERROR("串口未打开，无法开始通信");
        return;
    }

    ros::Publisher pub = nh.advertise<open_msgs::Serialmsg>(topic_name, loop_rate_hz);
    ros::Rate loop_rate(loop_rate_hz);
    ROS_INFO("wait for Frame");
    
    while (ros::ok()) {
        std::vector<uint8_t> frame;
        if (readFrame(frame)) {
            if (frame.size() >= 6) { // 至少包含帧头+长度+cmd1+cmd2+param1+param2+校验
                uint8_t frame_head = frame[0];
                uint8_t data_len   = frame[1];
                uint8_t cmd        = frame[2];
                uint8_t func       = frame[3];
                
                // 日志输出帧信息
                // std::ostringstream debug_oss;
                // debug_oss << "接收帧: ";
                // for (const auto& byte : frame) {
                //     debug_oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                // }
                // ROS_INFO_STREAM(debug_oss.str());
                // 只输出A5帧头数据
                if (frame_head != 0xA5) {
                    continue;
                }
                ROS_INFO("header: 0x%02x, length: %d, cmd: 0x%02x, func: 0x%02x", 
                        frame_head, data_len, cmd, func);
                
                // 创建并填充ROS消息
                open_msgs::Serialmsg msg;
                msg.frame_header = frame_head;
                msg.data_length = data_len;
                msg.cmd1 = cmd;
                msg.cmd2 = func;
                
                // 将数据部分放入data数组
                // 数据部分从索引4开始，直到倒数第二个字节(不包含校验和)
                if (data_len > 0) {
                    // 按照data_length分配空间
                    int data_length = data_len - 2; // 减去cmd1和cmd2的长度
                    msg.data.resize(data_length);
                    
                    // 只复制实际数据部分(cmd1和cmd2后的数据，不包括校验和)
                    size_t data_bytes = std::min(static_cast<size_t>(data_length), frame.size() - 5); // 4(帧头到cmd2) + 1(校验)
                    for (size_t i = 0; i < data_bytes; ++i) {
                        msg.data[i] = frame[i + 4]; // 从索引4(param1)开始
                    }
                }
                // 输出data内容
                std::ostringstream data_oss;
                data_oss << "data: ";
                for (const auto& byte : msg.data) {
                    data_oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                }
                ROS_INFO_STREAM(data_oss.str());
                
                // 校验和是最后一个字节
                msg.checksum = frame.back();
                
                // 发布消息
                pub.publish(msg);
            } else {
                ROS_WARN("data length is too short: %zu bytes", frame.size());
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}