#ifndef SERIAL_IO_HPP
#define SERIAL_IO_HPP

#include <string>

// 串口通信类（进阶要求2：向下位机发送yaw/pitch指令）
class SerialIO {
public:
    // 初始化串口（默认波特率115200，适配大多数下位机）
    SerialIO(const std::string& port = "/dev/ttyUSB0", int baudrate = 115200);
    ~SerialIO();
    // 发送yaw和pitch角度（报文格式：0xAA + yaw(4字节) + pitch(4字节) + 0x55）
    bool sendYawPitch(double yaw, double pitch);
    // 检查串口是否打开
    bool isOpen() const { return is_open_; }

private:
    int serial_handle_;  // 串口句柄（跨平台兼容）
    bool is_open_;         // 串口打开状态
};

#endif // SERIAL_IO_HPP