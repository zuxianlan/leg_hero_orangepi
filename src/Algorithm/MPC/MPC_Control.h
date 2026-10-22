/**
  ******************************************************************************
  * @file           : MPC_Control.h
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/17
  ******************************************************************************
  */
#ifndef MPC_CONTROL_H
#define MPC_CONTROL_H
/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
#include <qpOASES.hpp>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class MPC_Controller
{
public:
    Eigen::VectorXd x;
    Eigen::VectorXd x_ref;
    Eigen::MatrixXd u_out;

    Eigen::MatrixXd A; 
    Eigen::MatrixXd B; 
    Eigen::MatrixXd Q; 
    Eigen::MatrixXd R; 
    Eigen::MatrixXd F; 
    Eigen::MatrixXd Q_bar;
    Eigen::MatrixXd R_bar;

    Eigen::VectorXd u_min;
    Eigen::VectorXd u_max;

    Eigen::VectorXd x_min;
    Eigen::VectorXd x_max;

    MPC_Controller(int n, int p, int y, int N);
    void MPC_Init();

    void Set_x(const Eigen::VectorXd& x_);

    void BuildPredictionMatrices();

    void BuildHessianMatrices();

    void BuildGradientVector();

    void BuildConstraintBounds();

    void MPC_Calculate();

private:
    int n; 
    int p; 
    int y;
    int N; 
    int is_initialized;
    int solver_iterations{};

    qpOASES::QProblem qp;
    qpOASES::returnValue solver_status;

    Eigen::MatrixXd M; 
    Eigen::MatrixXd C; 

    Eigen::MatrixXd H; 
    Eigen::VectorXd g; 
    Eigen::MatrixXd A_mat; 
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
    Eigen::VectorXd lba;
    Eigen::VectorXd uba;

    Eigen::VectorXd last_control;
     // MPC 模型参数
    float dt_mpc = 0.002;
    float I_r = 1.5;
    float r = 0.16;
    float m = 25.0;
    float k = 0;
    float b = -1000;

    void initialize_qp();
};

#endif //MPC_CONTROL_H
