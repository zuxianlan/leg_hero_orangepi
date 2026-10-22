/**
  ******************************************************************************
  * @file           : bsp_libusb.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2026/1/16
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_libusb.h"
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

USBManager::USBManager(uint16_t vid, uint16_t pid,
                       uint8_t in_ep, uint8_t out_ep)
    : vid_(vid), pid_(pid), in_ep_(in_ep), out_ep_(out_ep)
{
}

USBManager::~USBManager()
{
    stop();
}

bool USBManager::start()
{
    if (running_) return true;

    // 初始化 libusb
    if (libusb_init(&ctx_) != 0)
    {
        std::cerr << "libusb init failed" << std::endl;
        return false;
    }

    // 设置热插拔回调
    libusb_hotplug_callback_handle handle;
    libusb_hotplug_register_callback(
        ctx_,
        LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
        LIBUSB_HOTPLUG_ENUMERATE,
        vid_, pid_,
        LIBUSB_HOTPLUG_MATCH_ANY,
        &USBManager::hotplug_cb,
        this,
        &handle
    );

    std::cout << "USB monitor started" << std::endl;
    running_ = true;

    // 启动事件线程
    event_thread_ = std::thread(&USBManager::event_thread, this);

    return true;
}

void USBManager::stop()
{
    if (!running_) return;

    running_ = false;

    if (event_thread_.joinable())
    {
        event_thread_.join();
    }

    close_device();

    if (ctx_)
    {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }

    std::cout << "USB monitor stopped" << std::endl;
}

void USBManager::event_thread() const
{
    while (running_)
    {
        struct timeval tv = {0, 100000}; // 100ms
        libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
    }
}

bool USBManager::open_device()
{
    handle_ = libusb_open_device_with_vid_pid(ctx_, vid_, pid_);
    if (!handle_) return false;

    // 声明接口0
    libusb_set_auto_detach_kernel_driver(handle_, 1);
    if (libusb_claim_interface(handle_, 0) != 0)
    {
        libusb_close(handle_);
        handle_ = nullptr;
        return false;
    }

    device_connected_ = true;

    // 创建3个异步传输
    for (int i = 0; i < 3; i++)
    {
        libusb_transfer* transfer = libusb_alloc_transfer(0);
        uint8_t* buffer = new uint8_t[1024];

        libusb_fill_bulk_transfer(
            transfer, handle_, in_ep_,
            buffer, 1024,
            &USBManager::transfer_cb,
            this, 1000
        );

        libusb_submit_transfer(transfer);
    }

    if (status_cb_) status_cb_(true);
    std::cout << "Device connected" << std::endl;

    return true;
}

void USBManager::close_device()
{
    if (!handle_) return;

    libusb_release_interface(handle_, 0);
    libusb_close(handle_);
    handle_ = nullptr;
    device_connected_ = false;

    if (status_cb_) status_cb_(false);
    std::cout << "Device disconnected" << std::endl;
}

bool USBManager::send(const uint8_t* data, int len)
{
    if (!device_connected_) return false;

    int transferred;
    int ret = libusb_bulk_transfer(
        handle_, out_ep_,
        const_cast<uint8_t*>(data), len,
        &transferred, 1000
    );

    return ret == 0 && transferred == len;
}

bool USBManager::is_connected() const
{
    return device_connected_;
}

// 静态回调函数
void USBManager::transfer_cb(libusb_transfer* transfer)
{
    auto* self = (USBManager*)transfer->user_data;

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
        transfer->actual_length > 0)
    {
        if (self->data_cb_)
        {
            self->data_cb_(transfer->buffer, transfer->actual_length);
        }
        else
        {
            // 默认打印
            std::cout << "Data: " << transfer->actual_length << " bytes" << std::endl;
        }
    }

    // 重新提交传输
    if (self->device_connected_)
    {
        libusb_submit_transfer(transfer);
    }
}

int USBManager::hotplug_cb(libusb_context* ctx, libusb_device* dev,
                           libusb_hotplug_event event, void* user)
{
    auto* self = (USBManager*)user;

    libusb_device_descriptor desc = {};
    libusb_get_device_descriptor(dev, &desc);

    if (desc.idVendor != self->vid_ || desc.idProduct != self->pid_)
    {
        return 0;
    }

    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
    {
        self->open_device();
    }
    else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT)
    {
        self->close_device();
    }

    return 0;
}
