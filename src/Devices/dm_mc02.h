/**
  ******************************************************************************
  * @file           : dm_mc02.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_DM_MC02_H
#define WHEEL_LEG_SYS_DM_MC02_H
/* Includes ------------------------------------------------------------------*/
#include "Bsp/BspSerialPort.h"
#include "bsp_libusb.h"
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

typedef struct
{
    float ch[5];
    float rotory_sw[3];
    char sw[5];
} RC_ctrl_t;

typedef struct
{
    float Position;
    float omega;

} Motor_data_t;

typedef struct
{
    uint8_t header;

    float gyro[3];
    float accel[3];
    float motionAccel_b[3];

    RC_ctrl_t RC_ctrl;

    Motor_data_t Motor_joint[4];
    Motor_data_t Motor_wheel[2];

    float roll;
    float pitch;
    float yaw;
    float motor_yaw_angle;

    // float current[2];
    uint16_t key;
} mc_02_Rx_data;

typedef struct
{
    uint8_t header;

    float joint_torque_set[4];
    float wheel_torque_set[2];
    uint8_t spin_flag;

} mc_02_Tx_data;

class mc_02_serial
{
public:
    mc_02_serial();
    ~mc_02_serial();

    USBManager usb = USBManager(0x0483, 0x5740);

    mc_02_Tx_data Tx_Data{};
    mc_02_Rx_data Rx_Data{};

    uint8_t Tx_buffer[64]{};

    // SerialPort serial_port_;

    int Init();
    std::mutex rx_mutex; // »¥³âËø
private:
};

#endif //WHEEL_LEG_SYS_DM_MC02_H
