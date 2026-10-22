/**
  ******************************************************************************
  * @file           : GamePad.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/12
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "GamePad.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class_GamePad::class_GamePad(std::string file_name) : file_name(std::move(file_name))
{

}

class_GamePad::~class_GamePad()
{
    Close();
}



bool class_GamePad::Open()
{
    // fd_ = open(file_name.c_str(), O_RDONLY);
    // if (fd_ < 0)
    // {
    //   perror("open");
    //   return -1;
    // }

    try
    {
        fd_ = open(file_name.c_str(), O_RDONLY | O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            throw std::runtime_error("Unable to open event port");
        }

        is_open_ = true;
        stop_read_thread_ = false;

        // 启动读取线程
        read_thread_ = std::thread(&class_GamePad::readThreadFunction, this);

        std::cout << "Successfully open event port: " << file_name << std::endl;
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

void class_GamePad::Close()
{
    // make sure event port is opened
    if (is_open_)
    {
        close(fd_);
        is_open_ = false;
        stop_read_thread_ = true;
    }
}

void class_GamePad::Receive_Data()
{
    if (type == XBOX_TYPE_BUTTON)
    {
        switch (code)
        {
        case XBOX_BUTTON_A:
            a = value;
            break;

        case XBOX_BUTTON_B:
            b = value;
            break;

        case XBOX_BUTTON_X:
            x = value;
            break;

        case XBOX_BUTTON_Y:
            y = value;
            break;

        case XBOX_BUTTON_LB:
            lb = value;
            break;

        case XBOX_BUTTON_RB:
            rb = value;
            break;

        case XBOX_BUTTON_FUNC:
            func = value;
            break;

        case XBOX_BUTTON_BACK:
            back = value;
            break;

        case XBOX_BUTTON_HOME:
            home = value;
            break;

        case XBOX_BUTTON_LO:
            lo = value;
            break;

        case XBOX_BUTTON_RO:
            ro = value;
            break;

        default:
            break;
        }
    }
    else if (type == XBOX_TYPE_AXIS)
    {
        switch (code)
        {
        case XBOX_AXIS_LX:
            lx = value;
            break;

        case XBOX_AXIS_LY:
            ly = value;
            break;

        case XBOX_AXIS_RX:
            rx = value;
            break;

        case XBOX_AXIS_RY:
            ry = value;
            break;

        case XBOX_AXIS_LT:
            lt = value;
            break;

        case XBOX_AXIS_RT:
            rt = value;
            break;

        case XBOX_AXIS_XX:
            xx = value;
            break;

        case XBOX_AXIS_YY:
            yy = value;
            break;

        default:
            break;
        }
    }
    else
    {
        /* Init do nothing */
    }
}

void class_GamePad::readThreadFunction()
{
    std::cout << "receiving event data" << std::endl;
    while (!stop_read_thread_)
    {
        // if event port is not opened, try open again
        if (!is_open_)
        {
            Open();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        int bytes_read = read(fd_, &event, sizeof(input_event));

        if (bytes_read > 0)
        {
            // receiving data
            type = event.type;
            code = event.code;
            value = event.value;
            Receive_Data();
        }
        else if (bytes_read < 0 && errno != EAGAIN)
        {
            std::cerr << "Failed to read data:" << strerror(errno) << std::endl;
            Close();
        }
    }

    // 短暂休眠
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

bool class_GamePad::IsOpen() const
{
    return is_open_;
}
