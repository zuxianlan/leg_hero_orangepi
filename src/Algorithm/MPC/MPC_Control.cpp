/**
  ******************************************************************************
  * @file           : MPC_Control.cpp
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/17
  ******************************************************************************
  */

/**
 * ============================================================================
 * 线性 MPC 实现说明
 * ----------------------------------------------------------------------------
 * 1. 离散模型
 *      x_{k+1} = A*x_k + B*u_k
 *
 * 2. 预测方程（把状态展开、消元）
 *      X = M*x0 + C*U
 *      X     : (N+1)*n = 64 维，未来 N+1 步状态
 *      U     : N*p     = 30 维，未来 N 步控制
 *      M     : (N+1)n x n  = 64 x 4
 *      C     : (N+1)n x Np = 64 x 30
 *
 * 3. 代价函数
 *      J = (X - X_ref)' * Q_bar * (X - X_ref) + U' * R_bar * U
 *        = U'*(C'*Q_bar*C + R_bar)*U
 *          + 2*(C'*Q_bar*(M*x0 - X_ref))'*U + 常数
 *
 * 4. 标准 QP
 *      min  1/2 * U'*H*U + g'*U
 *      s.t. lb <= U <= ub
 *      H = 2*(C'*Q_bar*C + R_bar)
 *      g = 2*C'*Q_bar*(M*x0 - X_ref)
 *
 *    注意：本文件为了少一次乘法，H/g 都没有乘 2。
 *    这等价于把整个目标函数乘了 1/2，最优解 U* 不变。
 *
 * 5. 求解
 *      第一次：qp.init(H, g, nullptr, lb, ub, nullptr, nullptr, nWSR)
 *      之后：  qp.hotstart(g, lb, ub, nullptr, nullptr, nWSR)
 *      然后：  qp.getPrimalSolution(u_out)
 *
 * 6. 本项目维度
 *      n=4, p=2, N=15
 *      M: 64x4, C: 64x30, Q_bar: 64x64, R_bar: 30x30
 *      H: 30x30, g: 30, lb/ub: 30
 *      QP 变量 30 个，约束只有 30 个上下界，没有一般约束。
 * ============================================================================
 */
/* Includes ------------------------------------------------------------------*/
#include "MPC_Control.h"
#include <iostream>
#include <numbers>
#include <algorithm>
#include <cmath>

/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;
using namespace qpOASES;


/* ---------------- 腿长数据集（与 HerKules_VOCAL_SJ_LQR_v5_14dim.m 的 Leg_data_l 一致）------- */
namespace {

constexpr int kLegN = 24;

// 第一列：腿长 L [m]；第二列：轮心到腿质心距离 l_w [m]
constexpr double kLegL[kLegN] = {
    0.1200,0.1317,0.1435,0.1552,0.1670,0.1787,0.1904,0.2022,
    0.2139,0.2257,0.2374,0.2491,0.2609,0.2726,0.2843,0.2961,
    0.3078,0.3196,0.3313,0.3430,0.3548,0.3665,0.3783,0.3900};
constexpr double kLegW[kLegN] = {
    0.10324,0.10653,0.10979,0.11295,0.11662,0.12042,0.12438,0.12851,
    0.13273,0.13732,0.14159,0.14606,0.15072,0.15541,0.16018,0.16505,
    0.17012,0.17493,0.17994,0.18499,0.19014,0.19529,0.20052,0.20576};

inline double LegClamp(double L) { return std::clamp(L, kLegL[0], kLegL[kLegN - 1]); }

inline int LegIndex(double L)
{
    int i = 0;
    while (i < kLegN - 2 && kLegL[i + 1] < L) ++i;
    return i;
}

// l_w(L)：分段线性插值
double LegLw(double L)
{
    L = LegClamp(L);
    const int i = LegIndex(L);
    const double t = (L - kLegL[i]) / (kLegL[i + 1] - kLegL[i]);
    return kLegW[i] + t * (kLegW[i + 1] - kLegW[i]);
}

// lambda = d(l_w)/dL：取当前区间的斜率
double LegLambda(double L)
{
    L = LegClamp(L);
    const int i = LegIndex(L);
    return (kLegW[i + 1] - kLegW[i]) / (kLegL[i + 1] - kLegL[i]);
}

} // namespace


/**
 * @brief 初始化 MPC 模型、权重、约束，并预计算与 x0 无关的矩阵
 *
 * 状态顺序（n=4）：
 *     x(0) = roll
 *     x(1) = roll_rate
 *     x(2) = height
 *     x(3) = height_rate
 *
 * 控制顺序（p=2）：
 *     u(0) = F_left
 *     u(1) = F_right
 *
 * 该函数只应在启动时调用一次（或模型参数变化后重新调用）。
 * 调用后会重建 M/C/H，并把 is_initialized 清 0，
 * 强制下一次 MPC_Calculate() 重新 init QP。
 */
void MPC_Controller::MPC_Init()
{
    //constexpr double kPi = std::numbers::pi;

    // 控制量限幅：左右腿力 ±150
    u_min << -150.0, -150.0;
    u_max <<  150.0,  150.0;

    // 状态限幅：目前只赋值，未加入 QP 约束
    x_min << -std::numbers::pi / 4.0, -0.5, 0.1, -1.0;
    x_max <<  std::numbers::pi / 4.0,  0.5, 0.38, 1.0;

    // 状态权重 Q：roll/height 大，速度项小
    Q << 1.5e7, 0.0, 0.0, 0.0,
         0.0, 1.0e3, 0.0, 0.0,
         0.0, 0.0, 1.5e7, 0.0,
         0.0, 0.0, 0.0, 1.0e3;

    // 终端权重 F：比 Q 大一个数量级
    F << 1.5e8, 0.0, 0.0, 0.0,
         0.0, 1.0e4, 0.0, 0.0,
         0.0, 0.0, 1.5e8, 0.0,
         0.0, 0.0, 0.0, 1.0e4;

    // 控制权重 R：左右腿力惩罚
    R << 0.9, 0.0,
         0.0, 0.9;

    // 4 维模型（横滚/高度）由腿长决定：先按标称腿长建一次，之后随腿长在线重建
    UpdateModel(0.18, 0.18);
}
/**
 * @brief 构造函数：分配矩阵维度并创建 QP 求解器
 *
 * 构造时 A/B/Q/R/F 还没有赋值，只建立正确维度的零矩阵/单位阵；
 * 真正的模型参数在 MPC_Init() 中设置，并会重新构建 M/C/H。
 *
 * @param n 状态维度（本项目 4）
 * @param p 控制维度（本项目 2）
 * @param y 输出维度（本项目未使用，传 0）
 * @param N 预测步数（本项目 15）
 */
MPC_Controller::MPC_Controller(const int n, const int p, const int y, const int N) : n(n), p(p), y(y), N(N)
{
    // x0：当前状态
    x = VectorXd::Zero(n);
    // 参考轨迹：(N+1) 个 n 维状态
    x_ref = VectorXd::Zero((N + 1) * n);
    // 最优控制序列：N 个 p 维控制
    u_out = VectorXd::Zero(N * p);

    // 维度先建好，具体值在 MPC_Init() 设置
    A = MatrixXd::Identity(n, n);
    B = MatrixXd::Zero(n, p);
    Q = MatrixXd::Zero(n, n);
    R = MatrixXd::Zero(p, p);
    F = MatrixXd::Zero(n, n);
    // 块对角权重矩阵
    Q_bar = MatrixXd::Identity((N + 1) * n, (N + 1) * n);
    R_bar = MatrixXd::Zero(N * p, N * p);

    // 约束向量
    x_min = VectorXd::Zero(n);
    x_max = VectorXd::Zero(n);
    u_min = VectorXd::Zero(p);
    u_max = VectorXd::Zero(p);

    // 预测矩阵
    M = MatrixXd::Zero((N + 1) * n, n);
    C = MatrixXd::Zero((N + 1) * n, N * p);

    // QP 数据
    H = MatrixXd::Zero(N * p, N * p);
    g = VectorXd::Zero(N * p);
    // 约束数据；当前只用 lb/ub
    A_mat = MatrixXd::Zero(N * (n + p), n + p);
    lb = VectorXd::Zero(N * (p));
    ub = VectorXd::Zero(N * (p));
    lba = VectorXd::Zero(N * (n + p));
    uba = VectorXd::Zero(N * (n + p));
    last_control = VectorXd::Zero(p);
    // 尚未初始化 QP
    is_initialized = 0;

    // qp = QProblem(N * p, 2 * N * (n +p));
    // 30 个变量，0 个一般约束
    qp = SQProblem(N * p, 0);
    Options options;
    // 实时运行可改为 PL_NONE
    options.printLevel = PL_LOW;
    // 初始化成非成功状态
    solver_status = TERMINAL_LIST_ELEMENT;
    // options.setToMPC();

    qp.setOptions(options);

    // 先用默认 A 建一次 M；MPC_Init() 会重建
    BuildPredictionMatrices();
}

/**
 * @brief 设置当前状态 x0
 *
 * 也可以直接给公有成员 x 赋值。
 */
void MPC_Controller::Set_x(const VectorXd& x_)
{
    x = x_;
}

/**
 * @brief 构建预测矩阵 M 和 C
 *
 * 预测方程：
 *     X = M*x0 + C*U
 *
 * M 的第 i 个块（i=0..N）：
 *     M_i = A^i
 *
 * C 的第 i 行块、第 j 列块（i=1..N, j=0..i-1）：
 *     C_{i,j} = A^(i-j-1) * B
 *
 * 维度：
 *     M: (N+1)n x n  = 64 x 4
 *     C: (N+1)n x Np = 64 x 30
 *
 * 这两个矩阵只与 A/B/N 有关，初始化时算一次即可。
 */
void MPC_Controller::BuildPredictionMatrices()
{

    // M 的第 0 块 = I
    M.block(0, 0, n, n) = MatrixXd::Identity(n, n);
    for (int i = 1; i <= N; i++)
    {
        // M_i = A * M_{i-1}
        M.block(i * n, 0, n, n) = A * M.block((i - 1) * n, 0, n, n);
    }

    // C 的第 0 行块为 0
    C.setZero();
    for (int i = 1; i <= N; i++)
    {
        for (int j = 0; j < i; j++)
        {

            // A^(i-j-1)
            MatrixXd A_pow = MatrixXd::Identity(n, n);
            for(int k=0; k < (i-j-1); k++) A_pow = A * A_pow;
            
            // C_{i,j} = A^(i-j-1) * B
            C.block(i * n, j * p, n, p) = A_pow * B;
        }
    }
}

/**
 * @brief 构建 Q_bar、R_bar 和 QP Hessian H
 *
 * Q_bar = blkdiag(Q, Q, ..., Q, F)   // 前 N 块 Q，最后一块终端权重 F
 * R_bar = blkdiag(R, R, ..., R)      // N 块 R
 * H     = C' * Q_bar * C + R_bar     // 30 x 30
 *
 * 注意：标准 QP 的 H 通常还要乘 2；这里没乘，
 * 等价于把目标函数整体缩放 1/2，不影响最优解。
 *
 * H 只与 A/B/Q/R/F 有关，初始化时算一次即可。
 */
void MPC_Controller::BuildHessianMatrices()
{
    Q_bar.setZero(); 
    for (int i = 0; i < N; i++)
    {
        // 前 N 块 Q
        Q_bar.block(i * n, i * n, n, n) = Q;
    }
    // 第 N 块终端权重 F
    Q_bar.block(N * n, N * n, n, n) = F; 

    R_bar.setZero();
    for (int i = 0; i < N; i++)
    {
        // 每步控制权重 R
        R_bar.block(i * p, i * p, p, p) = R;
    }

    // H = C'QbarC + Rbar
    H = (C.transpose() * Q_bar * C) + R_bar;
}


/**
 * @brief 构建 QP 梯度 g
 *
 * g = C' * Q_bar * (M*x0 - x_ref)
 *
 * M*x0 是“未来 N+1 步在控制量为 0 时的自由响应”，
 * 减去 x_ref 后是自由响应误差；C' * Q_bar 把它映射到控制空间。
 *
 * 维度：g = 30 x 1
 * 该函数每个控制周期都要调用，因为 x0 和 x_ref 会变。
 *
 * 注意：标准形式 g 还要乘 2；这里没乘，与 H 的缩放保持一致。
 */
void MPC_Controller::BuildGradientVector()
{
    // g = C'Qbar(Mx - xref)
    g = C.transpose() * Q_bar * (M * x - x_ref);
}

/**
 * @brief 构建控制量上下界 lb/ub
 *
 * 把 u_min/u_max 重复 N 次，得到 30 维向量：
 *     lb = [u_min; u_min; ...; u_min]
 *     ub = [u_max; u_max; ...; u_max]
 *
 * 当前 QP 没有一般约束，也没有状态约束：
 *     x_min/x_max 虽然设置了，但没有转成 A_mat/lba/uba 加入 QP。
 * 因此这是一个 box-constrained QP（只有上下界）。
 */
void MPC_Controller::BuildConstraintBounds()
{
   
    for (int i = 0; i < N; i++)
    {
        // 重复 N 次 u_min
        lb.segment(i * p, p) = u_min;
        // 重复 N 次 u_max
        ub.segment(i * p, p) = u_max;
    }
}

/**
 * @brief 第一次初始化 QP（或 hotstart 失败后重新初始化）
 *
 * qp.init 的参数顺序：
 *     H, g, A, lb, ub, lbA, ubA, nWSR
 *
 * 这里：
 *     A    = nullptr  // 没有一般约束
 *     lbA  = nullptr
 *     ubA  = nullptr
 *     nWSR = 100      // 最大 working set 迭代次数
 *
 * init 会建立初始工作集和矩阵分解；
 * 之后每个周期用 hotstart 复用这些信息。
 */
void MPC_Controller::initialize_qp()
{
    BuildGradientVector();
    BuildConstraintBounds();

    // 最大 working set 迭代次数
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

    // 记录实际迭代次数，便于调试
    solver_iterations = nWSR;

    // 下次可以 hotstart
    is_initialized = 1;
}

/**
 * @brief 每个控制周期调用一次：构建 g、求解 QP、输出 u_out
 *
 * 流程：
 *   1. 更新 g（x0/x_ref 变了）
 *   2. 更新 lb/ub（本项目是常量）
 *   3. 已初始化 -> hotstart；否则 -> init
 *   4. hotstart 失败 -> 重新 init
 *   5. 取最优解到 u_out
 *
 * u_out 是完整的最优控制序列（30 维）：
 *     [u0(0), u0(1), u1(0), u1(1), ..., u14(0), u14(1)]
 * 调用方只应使用前 p 个（u0），然后下一周期重新求解。
 */
/**
 * @brief 由腿长重建 4 维模型（横滚/高度），并重建 M/C/H
 *
 * 公式来源：HerKules_VOCAL_SJ_LQR_v5_14dim.m 的 Step 1.5
 *   先在 q=[l_l;l_r] 下写 M2*ddq + D2*dq + K2*q = [F_l;F_r]，再变换到 [alpha;H]
 *   M2 = 质量阵（机体 + 腿质心竖直动能 + 横滚惯量平行轴项）
 *   D2 = d_l*I（腿轴向粘性阻尼）
 *   K2 = -k_alpha/(4*R_leg^2)*[1 -1; -1 1]（重力负刚度，倒立摆）
 *   A4_pos = -T*M2^-1*K2*T^-1, A4_vel = -T*M2^-1*D2*T^-1, B4_acc = T*M2^-1
 *
 * 状态顺序 [roll, droll, height, dheight]，输入 [F_l, F_r]
 */
void MPC_Controller::UpdateModel(double L_l, double L_r)
{
    const double lwl = LegLw(L_l);
    const double lwr = LegLw(L_r);
    const double lam_l = LegLambda(L_l);
    const double lam_r = LegLambda(L_r);

    // ---- 14 维 Step 1.5：横滚/高度 4 维块 ----
    const double H0 = 0.5 * (L_l + L_r);
    const double I_alpha = I_roll
                         + m_b * (H0 + l_c) * (H0 + l_c)
                         + m_l * (lwl * lwl + lwr * lwr)
                         + m_l * R_leg * R_leg * (lam_l * lam_l + lam_r * lam_r);
    const double k_alpha = m_b * g_n * (H0 + l_c) + m_l * g_n * (lwl + lwr);
    const double M_ha = m_l * R_leg * (lam_r * lam_r - lam_l * lam_l);

    Eigen::Matrix2d J;
    J << 1.0, -1.0,
        -1.0, 1.0;

    Eigen::Matrix2d M2;
    M2 << m_b / 4.0 + m_l * lam_l * lam_l - M_ha / (2.0 * R_leg), m_b / 4.0,
          m_b / 4.0, m_b / 4.0 + m_l * lam_r * lam_r + M_ha / (2.0 * R_leg);
    M2 += (I_alpha / (4.0 * R_leg * R_leg)) * J;

    const Eigen::Matrix2d D2 = d_l * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d K2 = -(k_alpha / (4.0 * R_leg * R_leg)) * J;

    // [alpha; H] = T_ah * [l_l; l_r]
    Eigen::Matrix2d T_ah;
    T_ah << -1.0 / (2.0 * R_leg), 1.0 / (2.0 * R_leg),
             0.5,                  0.5;

    const Eigen::Matrix2d M2i = M2.inverse();
    const Eigen::Matrix2d T_ahi = T_ah.inverse();

    const Eigen::Matrix2d A4_pos = -T_ah * M2i * K2 * T_ahi;
    const Eigen::Matrix2d A4_vel = -T_ah * M2i * D2 * T_ahi;
    const Eigen::Matrix2d B4_acc =  T_ah * M2i;

    // ---- 连续模型 ----
    Eigen::Matrix4d Ac = Eigen::Matrix4d::Zero();
    Eigen::Matrix<double, 4, 2> Bc = Eigen::Matrix<double, 4, 2>::Zero();

    Ac(0, 1) = 1.0;
    Ac(2, 3) = 1.0;
    Ac(1, 0) = A4_pos(0, 0);  Ac(1, 2) = A4_pos(0, 1);
    Ac(1, 1) = A4_vel(0, 0);  Ac(1, 3) = A4_vel(0, 1);
    Ac(3, 0) = A4_pos(1, 0);  Ac(3, 2) = A4_pos(1, 1);
    Ac(3, 1) = A4_vel(1, 0);  Ac(3, 3) = A4_vel(1, 1);

    Bc(1, 0) = B4_acc(0, 0);  Bc(1, 1) = B4_acc(0, 1);
    Bc(3, 0) = B4_acc(1, 0);  Bc(3, 1) = B4_acc(1, 1);

    // ---- 离散化：二阶 Taylor（与 10 维 MPC 一致）----
    const double dt2 = double(dt_mpc) * double(dt_mpc);
    const double dtd = double(dt_mpc);
    const Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d Ad = I4 + Ac * dtd + 0.5 * Ac * Ac * dt2;
    const Eigen::Matrix<double, 4, 2> Bd =
        (I4 * dtd + 0.5 * Ac * dt2) * Bc;

    A = Ad;
    B = Bd;

    BuildPredictionMatrices();
    BuildHessianMatrices();

    model_l_ = L_l;
    model_r_ = L_r;
    model_ready_ = true;

    // 不重置 is_initialized：SQProblem::hotstart 带 H 参数，可直接用新 H 复用工作集
    //（比重开 QP 快很多）。若 hotstart 失败，MPC_Calculate 里会自动走 initialize_qp()。
}

void MPC_Controller::Set_Leg_Length(double L_l, double L_r)
{
    leg_l_ = L_l;
    leg_r_ = L_r;
}

/**
 * @brief 每个控制周期调用一次：按需重建模型 -> 构建 g -> 求解 QP -> 输出 u_out
 *
 * 与旧版的区别：
 *   1) 模型 A/B 不再是手写常数，而是由腿长在线重建（腿长变化 >2mm 时）
 *   2) H 随模型变化，求解器从 QProblem 换成 SQProblem，hotstart 要带上 H
 *   3) 按模型周期限速：调用快于 dt_mpc 时直接返回上次的解，保证"周期=模型 dt"
 */
void MPC_Controller::MPC_Calculate()
{
    // ---- 按模型周期限速（避免 1ms 调用、2ms 模型的不匹配）----
    const auto now = std::chrono::steady_clock::now();
    if (solved_once_ &&
        std::chrono::duration<double>(now - last_solve_).count() < 0.8 * double(dt_mpc))
    {
        return;   // 还没到下一次求解时刻，u_out 保持上一拍
    }
    last_solve_ = now;
    solved_once_ = true;

    // ---- 腿长变化 -> 重建模型 + H ----
    if (!model_ready_ ||
        std::fabs(leg_l_ - model_l_) > 2e-3 ||
        std::fabs(leg_r_ - model_r_) > 2e-3)
    {
        UpdateModel(leg_l_, leg_r_);
    }

    // 每周期更新 g
    BuildGradientVector();
    // 本项目 lb/ub 不变
    BuildConstraintBounds();

    // 已初始化：热启动
    if (is_initialized)
    {
        int nWSR = 100;

        // SQProblem：H 可能变化，hotstart 需要传入 H
        solver_status = qp.hotstart(
            H.data(), g.data(), nullptr,
            lb.data(), ub.data(),
            nullptr, nullptr, nWSR
        );

        // 失败则重新 init
        if (solver_status != qpOASES::SUCCESSFUL_RETURN)
        {
            initialize_qp();
        }
    }
    else
    {
        initialize_qp();
    }

    // 30 维最优解写入 u_out
    qp.getPrimalSolution(u_out.data());

    // 下周期可以 hotstart
    is_initialized = true;
}
