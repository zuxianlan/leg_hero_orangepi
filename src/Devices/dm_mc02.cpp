/**
  ******************************************************************************
  * @file           : dm_mc02.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dm_mc02.h"
#include <iostream>
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

mc_02_serial::mc_02_serial()
{
}

mc_02_serial::~mc_02_serial()
{

}

int mc_02_serial::Init()
{
    // ´ò¿ª´®¿Ú
    // if (!serial_port_.OpenPort())
    // {
    //     std::cerr << "Filed to open serial port" << std::endl;
    //     return -1;
    // }

    // std::cout << "Start to serial communication..." << std::endl;
    // return true;

    // ??????
    usb.on_status([](bool connected)
    {
        if (connected)
        {
            std::cout << "USB connected" << std::endl;
        }
        else
        {
            std::cout << "USB disconnected" << std::endl;
        }
    });

    // ??
    if (!usb.start())
    {
        std::cerr << "filed to start" << std::endl;
        return 1;
    }
    else
    {
        return 0;
    }
}
