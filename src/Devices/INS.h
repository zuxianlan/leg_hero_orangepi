/**
  ******************************************************************************
  * @file           : INS.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/4
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_INS_H
#define WHEEL_LEG_SYS_INS_H
/* Includes ------------------------------------------------------------------*/
#include "Algorithm/EKF/QuaternionEKF.h"
#include "Bsp/Timer.h"
#include "Algorithm/Fusion_AHRS/Tactical_Fusion.h"
#include "dm_mc02.h"
/* Define --------------------------------------------------------------------*/
#define correct_Time_define 1000
#define cheat 0

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class class_INS
{
public:

    float gyro[3]{};
    float accel[3]{};
    float MotionAccel_b[3]{};

    float roll{};
    float pitch{};
    float yaw{};

    void INS_Init();
    void INS_Update(const mc_02_Rx_data& rx);

    void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

    void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

};

#endif //WHEEL_LEG_SYS_INS_H
