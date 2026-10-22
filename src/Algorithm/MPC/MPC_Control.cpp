/**
  ******************************************************************************
  * @file           : MPC_Control.cpp
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/17
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MPC_Control.h"
#include <iostream>
#include <numbers>

/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;
using namespace qpOASES;

void MPC_Controller::MPC_Init()
{
    //constexpr double kPi = std::numbers::pi;

    u_min << -150.0, -150.0;
    u_max <<  150.0,  150.0;

    x_min << -std::numbers::pi / 4.0, -0.5, 0.1, -1.0;
    x_max <<  std::numbers::pi / 4.0,  0.5, 0.38, 1.0;

    Q << 1.5e7, 0.0, 0.0, 0.0,
         0.0, 1.0e3, 0.0, 0.0,
         0.0, 0.0, 1.5e7, 0.0,
         0.0, 0.0, 0.0, 1.0e3;

    F << 1.5e8, 0.0, 0.0, 0.0,
         0.0, 1.0e4, 0.0, 0.0,
         0.0, 0.0, 1.5e8, 0.0,
         0.0, 0.0, 0.0, 1.0e4;

    R << 0.9, 0.0,
         0.0, 0.9;

    A << 1.0, dt_mpc, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, dt_mpc,
         0.0, 0.0, 0.0, 1.0 - (dt_mpc * b / m);

    B << 0.0, 0.0,
         -(r * dt_mpc / I_r), r * dt_mpc / I_r,
         0.0, 0.0,
         dt_mpc / m, dt_mpc / m;

    // A/B/Q/R/F 都设置完之后再构建
    BuildPredictionMatrices();
    BuildHessianMatrices();

    // 如果之前已经初始化过 QP，参数改了要让它重新初始化
    is_initialized = 0;
}
/**
 * @brief MPC_Controller 构造函数
 * 
 * @param n 状态维度
 * @param p 控制维度
 * @param y 输出维度
 * @param N 预测步数
 */
MPC_Controller::MPC_Controller(const int n, const int p, const int y, const int N) : n(n), p(p), y(y), N(N)
{
    x = VectorXd::Zero(n);
    x_ref = VectorXd::Zero((N + 1) * n);
    u_out = VectorXd::Zero(N * p);

    A = MatrixXd::Identity(n, n);
    B = MatrixXd::Zero(n, p);
    Q = MatrixXd::Zero(n, n);
    R = MatrixXd::Zero(p, p);
    F = MatrixXd::Zero(n, n);
    Q_bar = MatrixXd::Identity((N + 1) * n, (N + 1) * n);
    R_bar = MatrixXd::Zero(N * p, N * p);

    x_min = VectorXd::Zero(n);
    x_max = VectorXd::Zero(n);
    u_min = VectorXd::Zero(p);
    u_max = VectorXd::Zero(p);

    M = MatrixXd::Zero((N + 1) * n, n);
    C = MatrixXd::Zero((N + 1) * n, N * p);

    H = MatrixXd::Zero(N * p, N * p);
    g = VectorXd::Zero(N * p);
    A_mat = MatrixXd::Zero(N * (n + p), n + p);
    lb = VectorXd::Zero(N * (p));
    ub = VectorXd::Zero(N * (p));
    lba = VectorXd::Zero(N * (n + p));
    uba = VectorXd::Zero(N * (n + p));
    last_control = VectorXd::Zero(p);
    is_initialized = 0;

    // qp = QProblem(N * p, 2 * N * (n +p));
    qp = QProblem(N * p, 0);
    Options options;
    options.printLevel = PL_LOW;
    solver_status = TERMINAL_LIST_ELEMENT;
    // options.setToMPC();

    qp.setOptions(options);

    BuildPredictionMatrices();
}

/**
 * @brief MPC_Controller 设置状态
 * 
 * @param x_ 状态向量
 */
void MPC_Controller::Set_x(const VectorXd& x_)
{
    x = x_;
}

/**
 * @brief MPC_Controller 构建预测矩阵
 * 
 */
void MPC_Controller::BuildPredictionMatrices()
{

    M.block(0, 0, n, n) = MatrixXd::Identity(n, n);
    for (int i = 1; i <= N; i++)
    {
        M.block(i * n, 0, n, n) = A * M.block((i - 1) * n, 0, n, n);
    }

    C.setZero();
    for (int i = 1; i <= N; i++)
    {
        for (int j = 0; j < i; j++)
        {

            MatrixXd A_pow = MatrixXd::Identity(n, n);
            for(int k=0; k < (i-j-1); k++) A_pow = A * A_pow;
            
            C.block(i * n, j * p, n, p) = A_pow * B;
        }
    }
}

/**
 * @brief MPC_Controller 构建海森矩阵
 * 
 */
void MPC_Controller::BuildHessianMatrices()
{
    Q_bar.setZero(); 
    for (int i = 0; i < N; i++)
    {
        Q_bar.block(i * n, i * n, n, n) = Q;
    }
    Q_bar.block(N * n, N * n, n, n) = F; 

    R_bar.setZero();
    for (int i = 0; i < N; i++)
    {
        R_bar.block(i * p, i * p, p, p) = R;
    }

    H = (C.transpose() * Q_bar * C) + R_bar;
}


/**
 * @brief MPC_Controller 构建梯度向量
 * 
 */
void MPC_Controller::BuildGradientVector()
{
    g = C.transpose() * Q_bar * (M * x - x_ref);
}

/**
 * @brief MPC_Controller 构建约束边界
 * 
 */
void MPC_Controller::BuildConstraintBounds()
{
   
    for (int i = 0; i < N; i++)
    {
        lb.segment(i * p, p) = u_min;
        ub.segment(i * p, p) = u_max;
    }
}

/**
 * @brief MPC_Controller 初始化QP问题
 * 
 */
void MPC_Controller::initialize_qp()
{
    BuildGradientVector();
    BuildConstraintBounds();

    int nWSR = 100;

    solver_status = qp.init(
        H.data(),
        g.data(),
        nullptr,
        lb.data(),
        ub.data(),
        nullptr,
        nullptr,
        nWSR);

    solver_iterations = nWSR;

    is_initialized = 1;
}

/**
 * @brief MPC_Controller 计算控制
 * 
 */
void MPC_Controller::MPC_Calculate()
{
    BuildGradientVector();
    BuildConstraintBounds();

    if (is_initialized)
    {
        int nWSR = 100;

        solver_status = qp.hotstart(
            g.data(), lb.data(), ub.data(),
            nullptr, nullptr, nWSR
        );

        if (solver_status != qpOASES::SUCCESSFUL_RETURN)
        {
            initialize_qp();
        }
    }
    else
    {
        initialize_qp();
    }

    qp.getPrimalSolution(u_out.data());

    is_initialized = true;

}
