/**
  ******************************************************************************
  * @file           : QuaternionEKF.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_QUATERNIONEKF_H
#define WHEEL_LEG_SYS_QUATERNIONEKF_H
/* Includes ------------------------------------------------------------------*/
#include "KalmanFilter.h"
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/
class class_Q_EKF
{
public:
    class_Q_EKF();
    void Quaternion_EKF_Init(float lambda, float dt);

    void Quaternion_EKF_Calculate(float gx, float gy, float gz, float ax, float ay, float az, float dt_);

    void QuaternionEKF_Linearization_F();

    void QuaternionEKF_Set_Linearization_H();

    void Quaternion_EKF_Set_linearization_Z();

    void Quaternion_Chi_Square_Check();

    void Quaternion_EKF_Limit();

    UserFunction UserFunc1{};
    UserFunction UserFunc2{};
    UserFunction UserFunc3{};
    UserFunction UserFunc4{};
    UserFunction UserFunc5{};

    float Roll = 0;
    float Pitch = 0;
    float Yaw = 0;
    float YawTotalAngle = 0;
    float Gyro[3] = {};
    float Accel[3] = {};

    float q[4] = {0}; // 四元数估计值

private:
    KalmanFilter kf;

    uint8_t Initialized = 0;
    uint8_t ConvergeFlag = 0;
    uint8_t StableFlag = 0;
    int16_t YawRoundCount = 0;
    uint64_t ErrorCount = 0;
    uint64_t UpdateCount = 0;
    
    float GyroBias[3] = {0}; // 陀螺仪零偏估计值
    float OrientationCosine[3] = {};

    float accLPFcoef = 0;
    float gyro_norm = 0;
    float accel_norm = 0;
    float AdaptiveGainScale = 0;

    float dt = 0; // 姿态更新周期
    float ChiSquareTestThreshold = 1e-8; // 卡方检验阈值
    float lambda = 0; // 渐消因子
    float YawAngleLast = 0;
    float r_k = 0;

    static float invSqrtf(float x);
};

void QuaternionEKF_Init(float lambda, float dt);

void Quaternion_EKF_Calculate(float gx, float gy, float gz, float ax, float ay, float az, float dt_);

float Get_Pitch();
float Get_Roll();
float Get_Yaw();
float Get_Total_Yaw();
float *Get_Accel();
float* Get_q();

#endif //WHEEL_LEG_SYS_QUATERNIONEKF_H
