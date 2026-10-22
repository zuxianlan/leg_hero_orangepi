/**
  ******************************************************************************
  * @file           : BspSerialPort.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "BspSerialPort.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <utility>
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

SerialPort::SerialPort(std::string port_name, int baud_rate) :
    baud_rate(baud_rate), port_name(std::move(port_name)),
    fd_(-1), is_open_(false), stop_read_thread_(false)
{

}

SerialPort::~SerialPort()
{
    ClosePort();
}

bool SerialPort::OpenPort()
{
    // 以读写方式打开串口，不控制终端（O_NOCTTY），非阻塞（O_NONBLOCK）
    try
    {
        fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ == -1)
        {
            throw std::runtime_error("Unable to open serial port");
        }

        // 恢复串口为阻塞状态
        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

        // 配置串口参数
        if (!SetParameters(baud_rate))
        {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("Unable to set parameters");
        }

        is_open_ = true;
        stop_read_thread_ = false;

        // 启动读取线程
        read_thread_ = std::thread(&SerialPort::readThreadFunction, this);

        std::cout << "Successfully open serial port: " << port_name << std::endl;
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
        return false;
    }
}

void SerialPort::ClosePort()
{
    if (is_open_)
    {
        // stop_read_thread_ = true;
        // if (read_thread_.joinable())
        // {
        //     read_thread_.join();
        // }

        close(fd_);
        fd_ = -1;
        is_open_ = false;
        std::cout << "Serial prot closed" << std::endl;
    }
}

void SerialPort::SetCallbackFunction(DataReceivedCallback callback)
{
    CallbackFunction = std::move(callback);
}

bool SerialPort::IsOpen() const
{
    return is_open_;
}

bool SerialPort::SetParameters(int baud_rate, int data_bits, int stop_bits, char parity) const
{
    struct termios options{};

    // 获取当前串口配置
    if (tcgetattr(fd_, &options) != 0)
    {
        std::cerr << "Failed to get serial port parameters" << std::endl;
        return false;
    }

    // 设置输入输出波特率
    speed_t speed = GetBaudRate(baud_rate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 设置控制模式
    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，接收使能

    // 设置数据位
    options.c_cflag &= ~CSIZE;
    switch (data_bits)
    {
    case 5: options.c_cflag |= CS5;
        break;
    case 6: options.c_cflag |= CS6;
        break;
    case 7: options.c_cflag |= CS7;
        break;
    case 8: options.c_cflag |= CS8;
        break;
    default: options.c_cflag |= CS8;
        break;
    }

    // 设置停止位
    if (stop_bits == 2)
    {
        options.c_cflag |= CSTOPB;
    }
    else
    {
        options.c_cflag &= ~CSTOPB;
    }

    // 设置校验位
    switch (parity)
    {
    case 'N':
    case 'n': // 无校验
        options.c_cflag &= ~PARENB;
        break;
    case 'E':
    case 'e': // 偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        break;
    case 'O':
    case 'o': // 奇校验
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
        break;
    default: // 默认无校验
        options.c_cflag &= ~PARENB;
        break;
    }

    // 设置输入模式
    options.c_iflag &= ~(INPCK | ISTRIP); // 关闭输入奇偶校验和去除第八位
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控制

    // 设置输出模式
    options.c_oflag &= ~OPOST; // 原始输出

    // 设置本地模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式

    // 设置等待时间和最小接收字符
    options.c_cc[VTIME] = 10; // 超时时间 (单位: 0.1秒)
    options.c_cc[VMIN] = 0; // 最小接收字符

    // 清除缓冲区并应用配置
    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &options) != 0)
    {
        std::cerr << "Failed to set serial port parameters" << std::endl;
        return false;
    }

    return true;
}

speed_t SerialPort::GetBaudRate(int baud_rate)
{
    switch (baud_rate)
    {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    default: return B115200;
    }
}

int SerialPort::send_data(const uint8_t* data, int length) const
{
    return write(fd_, data, length);
}

void SerialPort::readThreadFunction()
{
    std::cout << "receiving data" << std::endl;
    while (!stop_read_thread_)
    {
        // 如果没有打开串口则休眠
        if (!is_open_)
        {
            OpenPort();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        int bytes_read = read(fd_, Rx_Buffer, BUFFER_SIZE);

        if (bytes_read > 0)
        {
            // 如果有回调函数，则调用
            if (CallbackFunction)
            {
                std::vector<uint8_t> data(Rx_Buffer, Rx_Buffer + bytes_read);
                CallbackFunction(data);
            }
        }
        else if (bytes_read == 0 && errno != EAGAIN)
        {
            std::cerr << "Failed to read data:" << strerror(errno) << std::endl;
            ClosePort();
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // continue;
        }
    }

    // 短暂休眠
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
