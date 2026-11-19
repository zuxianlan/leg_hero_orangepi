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
#include <chrono>
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

    // 每拍写入当前腿长（A/B/H 随腿长重建）
    void Set_Leg_Length(double L_l, double L_r);

    // Riccati（终端代价 F）是否正常收敛
    bool RiccatiOK() const { return riccati_ok_; }

private:
    int n; 
    int p; 
    int y;
    int N; 
    int is_initialized;
    int solver_iterations{};

    qpOASES::SQProblem qp;
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

    // ===== 4 维模型参数（来自 HerKules_VOCAL_SJ_LQR_v5_14dim.m 的 Step 1.5）=====
    float dt_mpc = 0.002f;      // 离散步长 [s]，必须与实际求解周期一致
    float I_roll = 0.9f;        // 机体绕自身质心的横滚转动惯量 [kg m^2]  ★未实测，待标定
    float R_leg  = 0.22f;       // 左右腿受力线横向间距/2（横滚力臂）[m]
    float d_l    = 4.0f;        // 单腿沿腿方向等效粘性阻尼 [N s/m]      ★未辨识
    float l_c    = 0.037f;      // 机体质心到腿部关节中心点距离 [m]
    float m_b    = 35.542f;     // 机体质量 [kg]
    float m_l    = 2.6183599f;  // 单腿质量 [kg]
    float g_n    = 9.81f;       // 重力加速度 [m/s^2]

    double leg_l_ = 0.18, leg_r_ = 0.18;    // 外部每拍写入的当前腿长
    double model_l_ = 0.0, model_r_ = 0.0;  // 当前模型所用腿长
    bool   model_ready_ = false;            // 模型是否已建立
    bool   riccati_ok_  = false;            // F=Riccati 解是否收敛
    Eigen::MatrixXd P_riccati_;             // 上解出的 Riccati 解（热启动用）
    bool   solved_once_ = false;            // 是否已经求解过至少一次
    std::chrono::steady_clock::time_point last_solve_{};  // 上次求解时刻（按模型周期限速）

    // 由腿长重建 4 维模型（14 维 Step 1.5 的横滚/高度块）+ 离散化 + M/C/H
    void UpdateModel(double L_l, double L_r);

    void initialize_qp();
};

#endif //MPC_CONTROL_H
