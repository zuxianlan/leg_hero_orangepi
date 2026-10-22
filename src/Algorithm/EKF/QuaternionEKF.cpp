/**
  ******************************************************************************
  * @file           : QuaternionEKF.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "QuaternionEKF.h"
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;
class_Q_EKF Q_EKF;

void QuaternionEKF_Init(float lambda, float dt)
{
    Q_EKF.Quaternion_EKF_Init(lambda, dt);
}

void Quaternion_EKF_Calculate(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    Q_EKF.Quaternion_EKF_Calculate(gx, gy, gz, ax, ay, az, dt);
}

static void User_Function1()
{
    Q_EKF.QuaternionEKF_Linearization_F();
}

static void User_Function2()
{
    Q_EKF.QuaternionEKF_Set_Linearization_H();
}

static void User_Function3()
{
    Q_EKF.Quaternion_EKF_Set_linearization_Z();
}

static void User_Function4()
{
    Q_EKF.Quaternion_Chi_Square_Check();
}

static void User_Function5()
{
    Q_EKF.Quaternion_EKF_Limit();
}

class_Q_EKF::class_Q_EKF() : kf(6, 3, 0, User_Function1, User_Function2, User_Function3, User_Function4, User_Function5)
{
}

/**
 * @brief ��Ԫ��EKF��ʼ��
 *
 * @param lambda_ ��������
 * @param dt_ �������ڣ���λs
 * @return
 */
void class_Q_EKF::Quaternion_EKF_Init(float lambda_, float dt_)
{
    // ��ʼ����������
    kf.x_hat << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    kf.StateMinVariance << 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f;

    kf.z << 0.0f, 0.0f, 0.0f;

    kf.P << 100000.0f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
        0.1f, 100000.0f, 0.1f, 0.1f, 0.1f, 0.1f,
        0.1f, 0.1f, 100000.0f, 0.1f, 0.1f, 0.1f,
        0.1f, 0.1f, 0.1f, 100000.0f, 0.1f, 0.1f,
        0.1f, 0.1f, 0.1f, 0.1f, 100.0f, 0.1f,
        0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100.0f;

    kf.F << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    kf.H << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    
    constexpr float Q1 = 10.0f * 0.001f;
    constexpr float Q2 = 0.001f * 0.001f;
    constexpr float R1 = 8000000.0f;

    kf.Q << Q1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, Q1, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, Q1, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, Q1, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, Q2, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Q2;

    kf.R << R1, 0.0f, 0.0f,
        0.0f, R1, 0.0f,
        0.0f, 0.0f, R1;

    // ��ʼ������
    dt = dt_;
    lambda = lambda_;
}

void class_Q_EKF::Quaternion_EKF_Calculate(float gx, float gy, float gz, float ax, float ay, float az, float dt_)
{
    /*   F��������λ��,���ǺŵĲ��ּ�Ϊ�趨�����λ��
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */

    dt = dt_;

    // ��ʵ���ٶ�Ϊ�������ٶȼ���ƫ
    Gyro[0] = gx - GyroBias[0];
    Gyro[1] = gy - GyroBias[1];
    Gyro[2] = gz - GyroBias[2];

    // �趨F����
    volatile float halfgxdt = 0.5f * Gyro[0] * dt;
    volatile float halfgydt = 0.5f * Gyro[1] * dt;
    volatile float halfgzdt = 0.5f * Gyro[2] * dt;

    // �˲����趨״̬ת�ƾ���F�����Ͻǲ��� 4x4�Ӿ���,��0.5(Ohm-Ohm_bias)*deltaT,���½���һ��2x2��λ���Ѿ���ʼ������
    // ע����predict��F�����Ͻ���4x2�������,���ÿ��predict��ʱ�򶼻����memcpy�õ�λ�󸲸�ǰһ�����Ի���ľ�����predict���ֲ��������ϽǾ����������Ϊ��û�л�ô˽׶ε���Ԫ���������ݣ��������Ϊ���󣬵õ�������Ԫ�����ٽ��и���
    kf.F = MatrixXf::Identity(6, 6);

    kf.F(0, 1) = -halfgxdt;
    kf.F(0, 2) = -halfgydt;
    kf.F(0, 3) = -halfgzdt;

    kf.F(1, 0) = halfgxdt;
    kf.F(1, 2) = halfgzdt;
    kf.F(1, 3) = -halfgydt;

    kf.F(2, 0) = halfgydt;
    kf.F(2, 1) = -halfgzdt;
    kf.F(2, 3) = halfgxdt;

    kf.F(3, 0) = halfgzdt;
    kf.F(3, 1) = halfgydt;
    kf.F(3, 2) = -halfgxdt;

    // accel���ٶȽ��е�ͨ�˲�������ײ�����쳣��Ӱ��
    if (UpdateCount == 0) // ��һ�ν����ʼ��һ�¼��ٶ�
    {
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;
        UpdateCount++;
    }

    // һ�׵�ͨ�˲�
    Accel[0] = Accel[0] * accLPFcoef / (dt + accLPFcoef) + ax * dt / (dt + accLPFcoef);
    Accel[1] = Accel[1] * accLPFcoef / (dt + accLPFcoef) + ay * dt / (dt + accLPFcoef);
    Accel[2] = Accel[2] * accLPFcoef / (dt + accLPFcoef) + az * dt / (dt + accLPFcoef);

    // �趨��������������һ��
    accel_norm = sqrtf(Accel[0] * Accel[0]
        + Accel[1] * Accel[1]
        + Accel[2] * Accel[2]);
    volatile float accelInvNorm = 1.0f / accel_norm;

    kf.z(0) = Accel[0] * accelInvNorm;
    kf.z(1) = Accel[1] * accelInvNorm;
    kf.z(2) = Accel[2] * accelInvNorm;

    // �õ����ٶ�ģ��
    gyro_norm = sqrtf(Gyro[0] * Gyro[0]
        + Gyro[1] * Gyro[1]
        + Gyro[2] * Gyro[2]);

    // ������ٶ�С����ֵ�Ҽ��ٶȴ����趨��Χ��,��Ϊ�˶��ȶ�,���ٶȿ��������������ٶ�
    // �Ժ���������̬���²��ֻ�����StableFlag��ȷ��
    if (gyro_norm < 0.3f && accel_norm > 9.8f - 0.5f && accel_norm < 9.8f + 0.5f)
    {
        StableFlag = 1;
    }
    else
    {
        StableFlag = 0;
    }

    // ���ÿ������˲����㺯��
    kf.Predict();
    kf.Update();

    // �õ��������ݣ�������Ԫ������Ʈ
    q[0] = kf.x_hat(0);
    q[1] = kf.x_hat(1);
    q[2] = kf.x_hat(2);
    q[3] = kf.x_hat(3);
    GyroBias[0] = kf.x_hat(4);
    GyroBias[1] = kf.x_hat(5);
    GyroBias[2] = 0.0f; // �����ʱ��yaw�ᳯ�� �޷��۲�yawƯ��

    Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    Pitch = sinf(-2.0f * (-(q[0] * q[2]) + (q[1] * q[3])));
    Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));

    // ����yaw��Ȧ
    if (Yaw - YawAngleLast > std::numbers::pi)
    {
        YawRoundCount--;
    }
    else if (Yaw - YawAngleLast < -std::numbers::pi)
    {
        YawRoundCount++;
    }
    YawTotalAngle = YawRoundCount * 2 * static_cast<float>(std::numbers::pi) + Yaw;
    YawAngleLast = Yaw;
}

/**
 * @brief ��״̬ת�ƾ���F���Ͻ�4*2�ľ���������Ի���ͬʱ������Ʈ�����Э����
 *
 * @param
 * @return
 */
void class_Q_EKF::QuaternionEKF_Linearization_F()
{
    static float q0;
    static float q1;
    static float q2;
    static float q3;

    static float qInvNorm;

    q0 = kf.x_hat_p(0);
    q1 = kf.x_hat_p(1);
    q2 = kf.x_hat_p(2);
    q3 = kf.x_hat_p(3);

    qInvNorm = invSqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    // ��Ԫ����һ��
    for (uint8_t i = 0; i < 4; i++)
    {
        kf.x_hat_p(i) *= qInvNorm;
    }
    /*  F, ��*�ŵ��Ǵ˺����趨��ֵ
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    kf.F(0, 4) = (q1 * dt) / 2.0f;
    kf.F(0, 5) = (q2 * dt) / 2.0f;

    kf.F(1, 4) = -(q0 * dt) / 2.0f;
    kf.F(1, 5) = (q3 * dt) / 2.0f;

    kf.F(2, 4) = -(q3 * dt) / 2.0f;
    kf.F(2, 5) = -(q0 * dt) / 2.0f;

    kf.F(3, 4) = (q2 * dt) / 2.0f;
    kf.F(3, 5) = -(q1 * dt) / 2.0f;

    // ���Ͻ������ӣ���������Э�����ֹ��������
    kf.P(4, 4) /= lambda;
    kf.P(5, 5) /= lambda;

    // �޷�����ֹ��ɢ
    if (kf.P(4, 4) > 10000)
    {
        kf.P(4, 4) = 10000;
    }
    if (kf.P(5, 5) > 10000)
    {
        kf.P(5, 5) = 10000;
    }
}

/**
 * @brief ���ڸ���H�����ڹ����㴦��Jacobi����
 *
 * @param
 * @return
 */
void class_Q_EKF::QuaternionEKF_Set_Linearization_H()
{
    /* H����������ж���0
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    */
    static float double_q0;
    static float double_q1;
    static float double_q2;
    static float double_q3;

    double_q0 = 2.0f * kf.x_hat_p(0);
    double_q1 = 2.0f * kf.x_hat_p(1);
    double_q2 = 2.0f * kf.x_hat_p(2);
    double_q3 = 2.0f * kf.x_hat_p(3);

    kf.H.setZero();

    kf.H(0, 0) = -double_q2;
    kf.H(0, 1) = double_q3;
    kf.H(0, 2) = -double_q0;
    kf.H(0, 3) = double_q1;

    kf.H(1, 0) = double_q1;
    kf.H(1, 1) = double_q0;
    kf.H(1, 2) = double_q3;
    kf.H(1, 3) = double_q2;

    kf.H(2, 0) = double_q0;
    kf.H(2, 1) = -double_q1;
    kf.H(2, 2) = -double_q2;
    kf.H(2, 3) = double_q3;
}


/**
 * @brief �趨�����Ժ���H(X_p),���ڼ��㿨��У��
 *
 * @param
 * @return
 */
void class_Q_EKF::Quaternion_EKF_Set_linearization_Z()
{
    static float q0;
    static float q1;
    static float q2;
    static float q3;

    q0 = kf.x_hat_p(0);
    q1 = kf.x_hat_p(1);
    q2 = kf.x_hat_p(2);
    q3 = kf.x_hat_p(3);

    // ���²�������
    kf.z_nl(0) = 2.0f * (q1 * q3 - q0 * q2);
    kf.z_nl(1) = 2.0f * (q0 * q1 + q2 * q3);
    kf.z_nl(2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // ���������ķ�������
    for (uint8_t i = 0; i < 3; i++)
    {
        OrientationCosine[i] = cosf(fabsf(kf.z_nl(i)));
    }
}

/**
 * @brief ����У���жϴ���
 *
 * @param
 * @return
 */
void class_Q_EKF::Quaternion_Chi_Square_Check()
{
    // ����У���ж�

    // ����������
    kf.z_err = kf.z - kf.z_nl;

    // �����⺯��
    // D_k = H * P_p * H^T + R
    // r_k = z_err^T * D_k^-1 * z_err
    r_k = kf.z_err.transpose() * (kf.H * kf.P_p * kf.H.transpose() + kf.R).inverse() * kf.z_err;

    // rkС��0.5������ֵ���˲��������л��Ѿ�����
    if (r_k < 0.5f * ChiSquareTestThreshold)
    {
        ConvergeFlag = 1;
    }

    // rk�ϴ󣬵��˲�������������
    if (r_k > ChiSquareTestThreshold && ConvergeFlag)
    {
        if (StableFlag)
        {
            ErrorCount++; // ���徲ֹʱ���޷�ͨ����������
        }
        else
        {
            ErrorCount = 0;
        }

        if (ErrorCount > 50)
        {
            // �˲�����ɢ
            ConvergeFlag = 0;
        }
        else
        {
            // �в�δͨ���������飬��Ԥ��
            // x_hat = x_hat_p
            // P = P_p
            kf.K = MatrixXf::Zero(6, 3);
        }

        AdaptiveGainScale = 1;
    }
    else
    {
        // rkԽС������Խ�󣬷��������Ԥ��ֵ
        if (r_k > 0.1f * ChiSquareTestThreshold && ConvergeFlag)
        {
            AdaptiveGainScale = (ChiSquareTestThreshold - r_k) / (0.9f * ChiSquareTestThreshold);
        }
        else
        {
            AdaptiveGainScale = 1;
        }
        ErrorCount = 0;
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            kf.K(i, j) *= AdaptiveGainScale;
        }
    }
}


/**
 * @brief ������Ʈֵ��������yaw��������
 *
 * @param
 * @return
 */
void class_Q_EKF::Quaternion_EKF_Limit()
{
    if (ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf.x_k_out(i) > 0.01f * dt)
            {
                kf.x_k_out(i) = 0.01f * dt;
            }
            if (kf.x_k_out(i) < -0.01f * dt)
            {
                kf.x_k_out(i) = -0.01f * dt;
            }
        }
    }

    // ������yaw������
    // kf.x_k_out(3) = 0.0f;
}


float Get_Pitch()
{
    return Q_EKF.Pitch;
}

float Get_Roll()
{
    return Q_EKF.Roll;
}

float Get_Yaw()
{
    return Q_EKF.Yaw;
}

float Get_Total_Yaw()
{
    return Q_EKF.YawTotalAngle;
}

float* Get_Accel()
{
    return Q_EKF.Accel;
}

float* Get_q()
{
    return Q_EKF.q;
}


/**
 * @brief �Զ���1/sqrt(x),�ٶȸ���
 *
 * @param x x
 * @return float
 */
float class_Q_EKF::invSqrtf(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
