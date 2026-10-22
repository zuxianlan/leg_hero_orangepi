/**
  ******************************************************************************
  * @file           : KalmanFilter.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/9
  ******************************************************************************
  */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <vector>
#include <Eigen/Dense>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/
enum class KF_Mode
{
    Normal, // 正常
    Slip,   // 打滑
    Spin    // 小陀螺
};
/* Variable && Struct --------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

typedef void(*UserFunction)();

/**
* @brief 通用卡尔曼滤波器类，支持线性系统状态估计，可配置状态/观测/控制维度及用户自定义回调函数
*/
class KalmanFilter
{
private:
    int StateDim; 
    int MeasureDim; 
    int ControlDim; 

    UserFunction UserFunc1;
    UserFunction UserFunc2;
    UserFunction UserFunc3;
    UserFunction UserFunc4;
    UserFunction UserFunc5;


public:
    Eigen::MatrixXf F; 
    Eigen::MatrixXf B; 
    Eigen::MatrixXf H; 
    Eigen::MatrixXf Q; 
    Eigen::MatrixXf R; 
    Eigen::MatrixXf P; 
    Eigen::MatrixXf P_p; 
    Eigen::MatrixXf K; 
    Eigen::MatrixXf I; 

    Eigen::VectorXf x_hat_p; 
    Eigen::VectorXf x_k_out;
    Eigen::VectorXf x_hat;
    Eigen::VectorXf z; 
    Eigen::VectorXf z_nl; 
    Eigen::VectorXf z_err; 

    Eigen::VectorXf StateMinVariance; 

    /**
    * @brief 构造函数，初始化卡尔曼滤波器维度及用户自定义回调函数
    * @param StateDim 状态向量维度
    * @param MeasureDim 测量向量维度
    * @param ControlDim 控制向量维度
    * @param UserFunc1 用户自定义回调函数1，在预测阶段状态更新后调用
    * @param UserFunc2 用户自定义回调函数2，在预测阶段协方差更新后调用
    * @param UserFunc3 用户自定义回调函数3，在更新阶段开始时调用
    * @param UserFunc4 用户自定义回调函数4，在卡尔曼增益计算后调用
    * @param UserFunc5 用户自定义回调函数5，在状态修正量计算后调用
    */
    KalmanFilter(int StateDim, int MeasureDim, int ControlDim, UserFunction UserFunc1, UserFunction UserFunc2,
                 UserFunction UserFunc3, UserFunction UserFunc4, UserFunction UserFunc5);
    
    void KF_Init();
    void Set_Mode(KF_Mode mode);
    void Set_Z(float v, float a);
    void Predict(const Eigen::VectorXf& u = Eigen::VectorXf());
    void Update();

private:
};


#endif //KALMANFILTER_H