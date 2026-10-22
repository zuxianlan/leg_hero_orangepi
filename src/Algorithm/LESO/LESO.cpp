/**
  ******************************************************************************
  * @file           : LESO.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2026/6/6
  ******************************************************************************
  */
#include "LESO.h"
/* Includes ------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;

/**
 * @brief 构造函数
 * @param StateDim      原状态维度
 * @param InputDim      输入维度
 * @param OutputDim     输出维度
 * @param ExpansionDim  扩张状态维度 (通常取 OutputDim 或系统总扰动个数)
 * @param dt            离散化时间步长 (秒)
 */
Class_LESO::Class_LESO(int StateDim, int InputDim, int OutputDim, int ExpansionDim, float dt)
    : StateDim(StateDim), InputDim(InputDim), OutputDim(OutputDim),
      ExpansionDim(ExpansionDim), dt(dt)
{
    // 初始化矩阵为零矩阵，尺寸由外部 SetMatrices 和 SetObserverGain 确定
    // 此处仅预留空间，实际使用时需调用 SetMatrices 和 SetObserverGain
    A_e = MatrixXf::Zero(StateDim + ExpansionDim, StateDim + ExpansionDim);
    B_e = MatrixXf::Zero(StateDim + ExpansionDim, InputDim);
    C_e = MatrixXf::Zero(OutputDim, StateDim + ExpansionDim);
    L = MatrixXf::Zero(StateDim + ExpansionDim, OutputDim);
    
    // 初始化状态向量
    z_hat = VectorXf::Zero(StateDim + ExpansionDim);
    x_hat = VectorXf::Zero(StateDim);
    d_hat = VectorXf::Zero(ExpansionDim);
}

/**
 * @brief 初始化扩张状态估计值
 * @param z0 初始扩张状态向量，长度 = StateDim + ExpansionDim
 */
void Class_LESO::Init(const VectorXf& z0)
{
    if (z0.size() != StateDim + ExpansionDim)
    {
        std::cerr << "LESO::Init: dimension mismatch! Expected " 
                  << StateDim + ExpansionDim << ", got " << z0.size() << std::endl;
        return;
    }
    z_hat = z0;
    // 根据扩张状态结构分离原状态和扰动估计 (如果用户按 [x; d] 排列)
    x_hat = z_hat.head(StateDim);
    d_hat = z_hat.tail(ExpansionDim);
}

/**
 * @brief 设置扩张系统的状态矩阵、输入矩阵和输出矩阵
 * @param A_e 扩张状态矩阵 (StateDim+ExpansionDim) x (StateDim+ExpansionDim)
 * @param B_e 扩张输入矩阵 (StateDim+ExpansionDim) x InputDim
 * @param C_e 扩张输出矩阵 OutputDim x (StateDim+ExpansionDim)
 */
void Class_LESO::SetMatrices(const MatrixXf& A_e, const MatrixXf& B_e, const MatrixXf& C_e)
{
    // 维度检查
    int total_dim = StateDim + ExpansionDim;
    if (A_e.rows() != total_dim || A_e.cols() != total_dim)
    {
        std::cerr << "LESO::SetMatrices: A_e dimension error! Expected " 
                  << total_dim << "x" << total_dim << std::endl;
        return;
    }
    if (B_e.rows() != total_dim || B_e.cols() != InputDim)
    {
        std::cerr << "LESO::SetMatrices: B_e dimension error! Expected " 
                  << total_dim << "x" << InputDim << std::endl;
        return;
    }
    if (C_e.rows() != OutputDim || C_e.cols() != total_dim)
    {
        std::cerr << "LESO::SetMatrices: C_e dimension error! Expected " 
                  << OutputDim << "x" << total_dim << std::endl;
        return;
    }
    
    this->A_e = A_e;
    this->B_e = B_e;
    this->C_e = C_e;
}

/**
 * @brief 设置观测器增益矩阵
 * @param L 观测器增益矩阵 (StateDim+ExpansionDim) x OutputDim
 */
void Class_LESO::SetObserverGain(const MatrixXf& L)
{
    int total_dim = StateDim + ExpansionDim;
    if (L.rows() != total_dim || L.cols() != OutputDim)
    {
        std::cerr << "LESO::SetObserverGain: L dimension error! Expected " 
                  << total_dim << "x" << OutputDim << std::endl;
        return;
    }
    this->L = L;
}

/**
 * @brief 执行一步离散观测器更新
 * @param u 当前输入向量，长度 = InputDim
 * @param y 当前测量输出向量，长度 = OutputDim
 * 
 * 观测器方程：z(k+1) = A_e * z(k) + B_e * u(k) + L * (y(k) - C_e * z(k))
 */
void Class_LESO::Update(const VectorXf& u, const VectorXf& y)
{
    // 输入输出维度检查
    if (u.size() != InputDim)
    {
        std::cerr << "LESO::Update: u dimension error! Expected " 
                  << InputDim << ", got " << u.size() << std::endl;
        return;
    }
    if (y.size() != OutputDim)
    {
        std::cerr << "LESO::Update: y dimension error! Expected " 
                  << OutputDim << ", got " << y.size() << std::endl;
        return;
    }
    
    // 计算输出预测误差
    VectorXf y_pred = C_e * z_hat;
    VectorXf err = y - y_pred;
    
    // 离散观测器递推 (欧拉离散化，假设矩阵已离散化或步长已包含)
    // 若用户提供的是连续时间矩阵，可改为：z_hat += dt * (A_e * z_hat + B_e * u + L * err)
    // 此处默认用户已提供离散化矩阵，直接一步递推
    VectorXf z_next = A_e * z_hat + B_e * u + L * err;
    
    // 更新状态
    z_hat = z_next;
    
    // 分离原状态和扰动估计 (假设状态向量前 StateDim 个为原状态，后 ExpansionDim 个为扰动)
    x_hat = z_hat.head(StateDim);
    d_hat = z_hat.tail(ExpansionDim);
}

/**
 * @brief 获取原状态估计
 * @return 原状态向量 (长度 StateDim)
 */
VectorXf Class_LESO::get_state_estimate() const
{
    return x_hat;
}

/**
 * @brief 获取扰动估计
 * @return 扰动向量 (长度 ExpansionDim)
 */
VectorXf Class_LESO::get_disturbance_estimate() const
{
    return d_hat;
}

/**
 * @brief 获取完整扩张状态估计
 * @return 扩张状态向量 (长度 StateDim+ExpansionDim)
 */
VectorXf Class_LESO::get_full_estimate() const
{
    return z_hat;
}

/**
 * @brief 手动设置当前扩张状态估计
 * @param z_new 新的扩张状态向量
 */
void Class_LESO::set_estimate(const VectorXf& z_new)
{
    if (z_new.size() != StateDim + ExpansionDim)
    {
        std::cerr << "LESO::set_estimate: dimension mismatch! Expected " 
                  << StateDim + ExpansionDim << ", got " << z_new.size() << std::endl;
        return;
    }
    z_hat = z_new;
    x_hat = z_hat.head(StateDim);
    d_hat = z_hat.tail(ExpansionDim);
}