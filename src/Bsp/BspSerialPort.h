/**
  ******************************************************************************
  * @file           : BspSerialPort.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_BSPSERIALPORT_H
#define WHEEL_LEG_SYS_BSPSERIALPORT_H

/* Includes ------------------------------------------------------------------*/
#include <string>
#include <termios.h>
#include <thread>
#include <functional>
#include <atomic>
#include <vector>
/* Define --------------------------------------------------------------------*/

// 缓冲区字节长度
#define BUFFER_SIZE 512

// 定义回调函数类型
using DataReceivedCallback = std::function<void(std::vector<uint8_t>&)>;

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/



class SerialPort
{
public:
    int baud_rate = B115200;
    std::string port_name;
    explicit SerialPort(std::string port_name,int baud_rate) ;
    ~SerialPort();

    // 打开串口
    bool OpenPort();

    // 关闭串口
    void ClosePort();

    // 发送数据
    int send_data(const uint8_t* data, int length) const;

    // 检查串口是否打开
    [[nodiscard]] bool IsOpen() const;

    // 设置串口参数
    [[nodiscard]] bool SetParameters(int baud_rate, int data_bits = 8, int stop_bits = 1, char parity = 'N') const;

    // 设置数据接收回调函数
    void SetCallbackFunction(DataReceivedCallback Callback);

private:
    // 读取线程函数
    void readThreadFunction();

    int fd_; // 串口文件描述符
    std::atomic<bool> is_open_;
    std::atomic<bool> stop_read_thread_;
    std::thread read_thread_;
    DataReceivedCallback CallbackFunction;

    uint16_t Rx_length = 512;
    uint8_t Tx_Buffer[512]{};
    uint8_t Rx_Buffer[512]{};

    // 设置波特率
    static speed_t GetBaudRate(int baud_rate);
};


#endif //WHEEL_LEG_SYS_BSPSERIALPORT_H
