#include "serial_io.hpp"
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#endif

// 初始化串口（跨平台实现）
SerialIO::SerialIO(const std::string& port, int baudrate) : serial_handle_(-1), is_open_(false) {
#ifdef _WIN32
    // Windows系统串口初始化
    serial_handle_ = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                                 OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (serial_handle_ == INVALID_HANDLE_VALUE) {
        std::cerr << "[SerialIO] Windows串口打开失败：" << port << std::endl;
        return;
    }
    // 配置串口参数
    DCB dcb;
    GetCommState((HANDLE)serial_handle_, &dcb);
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    SetCommState((HANDLE)serial_handle_, &dcb);
    // 设置超时
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts((HANDLE)serial_handle_, &timeouts);
#else
    // Linux系统串口初始化
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "[SerialIO] Linux串口打开失败：" << port << std::endl;
        return;
    }
    serial_handle_ = fd;
    // 配置串口参数
    struct termios options;
    tcgetattr(fd, &options);
    // 设置波特率
    switch (baudrate) {
        case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
        case 9600: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break;
        default: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200);
    }
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    tcsetattr(fd, TCSANOW, &options);
#endif
    is_open_ = true;
    std::cout << "[SerialIO] 串口初始化成功：" << port << "（波特率：" << baudrate << "）" << std::endl;
}

// 释放串口资源
SerialIO::~SerialIO() {
#ifdef _WIN32
    if (serial_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle((HANDLE)serial_handle_);
    }
#else
    if (serial_handle_ != -1) {
        close(serial_handle_);
    }
#endif
}

// 发送yaw和pitch角度
bool SerialIO::sendYawPitch(double yaw, double pitch) {
    if (!is_open_) {
        std::cerr << "[SerialIO] 串口未打开，发送失败！" << std::endl;
        return false;
    }
    // 构建报文（帧头0xAA + 数据 + 帧尾0x55）
    uint8_t buf[10] = {0xAA};
    memcpy(buf + 1, &yaw, 4);    // yaw（4字节浮点数）
    memcpy(buf + 5, &pitch, 4);  // pitch（4字节浮点数）
    buf[9] = 0x55;               // 帧尾

    // 发送数据
#ifdef _WIN32
    DWORD bytes_written;
    WriteFile((HANDLE)serial_handle_, buf, sizeof(buf), &bytes_written, nullptr);
    return bytes_written == sizeof(buf);
#else
    ssize_t bytes_written = write(serial_handle_, buf, sizeof(buf));
    return bytes_written == sizeof(buf);
#endif
}