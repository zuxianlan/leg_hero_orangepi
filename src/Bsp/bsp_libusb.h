/**
  ******************************************************************************
  * @file           : bsp_libusb.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2026/1/16
  ******************************************************************************
  */
#ifndef LIBUSB_TEST_BSP_LIBUSB_H
#define LIBUSB_TEST_BSP_LIBUSB_H
/* Includes ------------------------------------------------------------------*/
#include "libusb-1.0/libusb.h"
#include <iostream>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <cstring>
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/




class USBManager {
public:
    // 数据回调类型: (数据指针, 数据长度)
    using DataCallback = std::function<void(const uint8_t*, int)>;

    // 状态回调类型: (是否连接)
    using StatusCallback = std::function<void(bool)>;

    // 构造函数
    explicit USBManager(uint16_t vid = 0x1234, uint16_t pid = 0x5678,
               uint8_t in_ep = 0x81, uint8_t out_ep = 0x01);

    ~USBManager();

    // 主函数
    bool start();            // 开始监听
    void stop();             // 停止
    bool is_connected() const;     // 检查连接状态
    bool send(const uint8_t* data, int len);  // 发送数据

    // 设置回调
    void on_data(DataCallback cb) { data_cb_ = cb; }
    void on_status(StatusCallback cb) { status_cb_ = cb; }

private:
    // libusb 回调
    static void transfer_cb(libusb_transfer* transfer);
    static int hotplug_cb(libusb_context* ctx, libusb_device* dev,
                          libusb_hotplug_event event, void* user);

    // 内部函数
    bool open_device();
    void close_device();
    void event_thread() const;

    // 配置
    uint16_t vid_, pid_;
    uint8_t in_ep_, out_ep_;

    // 状态
    bool running_ = false;
    bool device_connected_ = false;

    // libusb 对象
    libusb_context* ctx_ = nullptr;
    libusb_device_handle* handle_ = nullptr;

    // 回调
    DataCallback data_cb_;
    StatusCallback status_cb_;

    // 线程
    std::thread event_thread_;
};


#endif //LIBUSB_TEST_BSP_LIBUSB_H
