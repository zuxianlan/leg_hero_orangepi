/**
  ******************************************************************************
  * @file           : LESO.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2026/6/6
  ******************************************************************************
  */
#ifndef LESO_TEST_LESO_H
#define LESO_TEST_LESO_H
/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <Eigen/Dense>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class Class_LESO
{
private:
  // 系统维度
  int StateDim;          // 原状态维度 x
  int InputDim;          // 输入维度 u
  int OutputDim;         // 输出维度 y
  int ExpansionDim;      // 扩张状态维度 (通常等于输出维度或用户定义)

  // 离散观测器矩阵
  Eigen::MatrixXf A_e;   // 扩张系统状态矩阵 (StateDim+ExpansionDim) x (StateDim+ExpansionDim)
  Eigen::MatrixXf B_e;   // 扩张系统输入矩阵 (StateDim+ExpansionDim) x InputDim
  Eigen::MatrixXf C_e;   // 扩张系统输出矩阵 OutputDim x (StateDim+ExpansionDim)
  Eigen::MatrixXf L;     // 观测器增益矩阵 (StateDim+ExpansionDim) x OutputDim

  // 状态变量
  Eigen::VectorXf z_hat; // 扩张状态估计向量，长度 = StateDim + ExpansionDim
  Eigen::VectorXf x_hat; // 原状态估计，长度 = StateDim
  Eigen::VectorXf d_hat; // 扰动估计，长度 = ExpansionDim

  // 时间步长
  float dt;              // 离散化步长 (秒)

public:
  // 构造函数
  Class_LESO(int StateDim, int InputDim, int OutputDim, int ExpansionDim, float dt);

  // 初始化估计状态
  void Init(const Eigen::VectorXf& z0);

  // 设置扩张系统矩阵 (用户需提前离散化，或提供连续矩阵配合dt)
  void SetMatrices(const Eigen::MatrixXf& A_e, const Eigen::MatrixXf& B_e,
                   const Eigen::MatrixXf& C_e);

  // 设置观测器增益
  void SetObserverGain(const Eigen::MatrixXf& L);

  // 一步更新：根据当前输入u和测量输出y，更新扩张状态估计
  void Update(const Eigen::VectorXf& u, const Eigen::VectorXf& y);

  // 获取原状态估计
  Eigen::VectorXf get_state_estimate() const;

  // 获取扰动估计
  Eigen::VectorXf get_disturbance_estimate() const;

  // 获取完整扩张状态估计
  Eigen::VectorXf get_full_estimate() const;

  // 手动设置当前估计值 (用于重置或外部干预)
  void set_estimate(const Eigen::VectorXf& z_new);
};



#endif //LESO_TEST_LESO_H
