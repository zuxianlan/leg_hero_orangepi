/**
  ******************************************************************************
  * @file           : KalmanFilter.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/9
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "KalmanFilter.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;

/**
* @brief 构造函数，初始化卡尔曼滤波器各矩阵为适当维度，并注册用户自定义回调函数
*/
KalmanFilter::KalmanFilter(int StateDim, int MeasureDim, int ControlDim = 0, UserFunction UserFunc1 = nullptr,
                           UserFunction UserFunc2 = nullptr, UserFunction UserFunc3 = nullptr,
                           UserFunction UserFunc4 = nullptr,
                           UserFunction UserFunc5 = nullptr) : StateDim(StateDim),
                                                               MeasureDim(MeasureDim),
                                                               ControlDim(ControlDim), UserFunc1(UserFunc1),
                                                               UserFunc2(UserFunc2),
                                                               UserFunc3(UserFunc3), UserFunc4(UserFunc4),
                                                               UserFunc5(UserFunc5)
{
    F = MatrixXf::Identity(StateDim, StateDim);
    H = MatrixXf::Zero(MeasureDim, StateDim);
    Q = MatrixXf::Zero(StateDim, StateDim);
    R = MatrixXf::Zero(MeasureDim, MeasureDim);
    P = MatrixXf::Identity(StateDim, StateDim);
    P_p = MatrixXf::Identity(StateDim, StateDim);
    K = MatrixXf::Zero(StateDim, MeasureDim);
    I = MatrixXf::Identity(StateDim, StateDim);

    x_hat_p = VectorXf::Zero(StateDim);
    x_k_out = VectorXf::Zero(StateDim);
    x_hat = VectorXf::Zero(StateDim);
    StateMinVariance = VectorXf::Zero(StateDim);
    z = VectorXf::Zero(MeasureDim);
    z_nl = VectorXf::Zero(MeasureDim);
    z_err = VectorXf::Zero(MeasureDim);

    if (ControlDim > 0)
    {
        B = MatrixXf::Zero(StateDim, ControlDim);
    }
}

/**
* @brief 2状态底盘速度卡尔曼滤波器的专用初始化，设置状态初值、协方差及系统矩阵，仅当StateDim=2且MeasureDim=2时有效
*/
void KalmanFilter::KF_Init()
{
    if (StateDim != 2 || MeasureDim != 2)
    {
        std::cerr << "KF_Init: only supports 2-state/2-measure KF" << std::endl;
        return;
    }

    x_hat << 0.0f, 0.0f;
    StateMinVariance << 0.0001f, 0.0300f;

    P << 1.0f, 0.0f,
         0.0f, 1.0f;

    // dt = 1ms
    F << 1.0f, 0.001f,
         0.0f, 1.0f;

    H << 1.0f, 0.0f,
         0.0f, 1.0f;

    Set_Mode(KF_Mode::Normal);
}

/**
* @brief 动态切换滤波器工作模式，根据不同模式自动设置对应的Q/R噪声矩阵参数
* @param mode 滤波器工作模式（Normal:正常 / Slip:打滑 / Spin:小陀螺）
*/
void KalmanFilter::Set_Mode(KF_Mode mode)
{
    if (StateDim != 2 || MeasureDim != 2) return;   //维度不是2直接返回
    switch (mode)
    {
    case KF_Mode::Normal:
        Q << 1.0f, 0.0f,
             0.0f, 2000.0f;

        R << 100.0f, 0.0f,
             0.0f, 1.0f;
        break;

    case KF_Mode::Slip:
        Q << 0.1f, 0.0f,
             0.0f, 4000.0f;

        R << 9000.0f, 0.0f,
             0.0f, 100.0f;
        break;

    case KF_Mode::Spin:
        Q << 1.0f, 0.0f,
             0.0f, 1.0f;

        R << 100.0f, 0.0f,
             0.0f, 8000.0f;
        break;
    }
}

/**
* @brief 直接设置2维测量向量z = [速度, 加速度]
* @param v 速度测量值
* @param a 加速度测量值
*/
void KalmanFilter::Set_Z(float v, float a)
{
    if (z.size() != 2) return;
    z << v, a;
}

/**
* @brief 卡尔曼滤波预测步骤：计算先验状态估计x_hat_p和先验协方差矩阵P_p，并依次调用用户回调函数1和2
* @param u 控制输入向量（可选，仅当ControlDim>0且u非空时参与计算）
*/
void KalmanFilter::Predict(const VectorXf& u)
{
    // 1. X_k = F * X_k-1 + B * U_k-1
    if (u.size() > 0 && ControlDim > 0)
    {
        x_hat_p = F * x_hat + B * u;
    }
    else
    {
        x_hat_p = F * x_hat;
    }

    if (UserFunc1 != nullptr)
    {
        UserFunc1();
    }

    // 2. P_kp = F * P_k-1 * F^T + Q
    P_p = F * P * F.transpose() + Q;

    if (UserFunc2 != nullptr)
    {
        UserFunc2();
    }
}

/**
* @brief 卡尔曼滤波更新步骤：计算卡尔曼增益K，更新后验状态估计x_hat和后验协方差矩阵P，并对协方差对角线进行下界约束
*/
void KalmanFilter::Update()
{
    if (UserFunc3 != nullptr)
    {
        UserFunc3();
    }

    // 3.K = (P_kp * H^T) / (H * P_kp * H^T + R)
    K = (P_p * H.transpose()) * (H * P_p * H.transpose() + R).inverse();

    if (UserFunc4 != nullptr)
    {
        UserFunc4();
    }

    x_k_out = K * (z - H * x_hat_p);

    if (UserFunc5 != nullptr)
    {
        UserFunc5();
    }

    // 4.X_hat = X_hat_p + K * (Z - H * X_hat_p)
    x_hat = x_hat_p + x_k_out;

    // 5.P = (I - K * H) * P_p
    P = (I - K * H) * P_p;

    for (int i = 0; i < StateDim; i++)
    {
        if (P(i, i) < StateMinVariance(i))
        {
            P(i, i) = StateMinVariance(i);
        }
    }
}