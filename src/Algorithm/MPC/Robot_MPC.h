/**
  ******************************************************************************
  * @file           : Robot_MPC.h
  * @author         : Zhu_hechen
  * @brief          : 机体全状态线性MPC控制
  * @attention      : 状态用「误差态」e = x - x_ref，与现 LQR 的 err[10] 完全同构
  * @date           : 2026/10/27
  ******************************************************************************
  *
  *  状态 e(10): [s, ds, phi, dphi, th_ll, dth_ll, th_lr, dth_lr, th_b, dth_b]
  *  控制 u(4) : [T_wl, T_wr, T_bl, T_br]   —— 与现 LQR 的 T[0..3] 一一对应
  *
  *  每周期（MPC_CFG::DT = 8 ms）:
  *      g = CtQ · Mp · e    ──>   QP   ──>   U(60)   ──>   只取前 4 个
  *
  *  腿长变化 >2 mm 或 100 ms 一次:
  *      5x5 求 A_c,B_c -> ZOH 离散 A_d,B_d -> Riccati 得 F(=P),K
  *      -> condensing(Mp,Cp) -> H -> qp.init
  *
  *  注意: 本文件刻意不使用标识符 N，因为 Chassis.h 里已有 #define N 15。
  ******************************************************************************
  */
#ifndef ROBOT_MPC_H
#define ROBOT_MPC_H

#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <memory>
#include <algorithm>
#include <cmath>


// ============================================================================
//  ★★★ 约束集中配置区：所有约束只改这一段 ★★★
// ============================================================================
namespace MPC_CFG
{
    // ---- 0) MPC 周期 [s]：定时器周期、模型离散步长、输出超时都由它决定 ----
    constexpr double DT = 0.008;

    // ---- 1) 力矩盒约束 [T_wl, T_wr, T_bl, T_br]，单位 Nm ----
    constexpr double U_MIN[4] = {-10.0,  -10.0, -45.0, -45.0};
    constexpr double U_MAX[4] = { 10.0,   10.0,  45.0,  45.0};

    // ---- 2) 力矩变化率，单位 Nm / 每步(=DT) ----
    constexpr double DU[4] = {2.0, 2.0, 5.0, 5.0};

    // ---- 3) 摆角限幅随腿长线性变化（腿越长，允许摆角越小）----
    //   最短腿 LEG_MIN -> ±THETA_AT_MIN
    //   最长腿 LEG_MAX -> ±THETA_AT_MAX
    //   ★若方向反了，把这两个值对调即可
    constexpr double LEG_MIN      = 0.107;
    constexpr double LEG_MAX      = 0.39;
    constexpr double THETA_AT_MIN = 0.40;    // L = 0.107 时的摆角限幅
    constexpr double THETA_AT_MAX = 0.15;    // L = 0.39 时的摆角限幅

    // ---- 4) 机体俯仰限幅，单位 rad ----
    constexpr double PITCH_MAX = 0.30;

    // ---- 4b) 摆杆角速度限幅，单位 rad/s ----
    constexpr double DTHETA_MAX = 2.0;      // |dθ_ll|, |dθ_lr| 上限

    // ---- 5) 状态约束（3/4/4b）为软约束：每个约束一个松弛量 s >= 0，整个时域共用 ----
    //   约束变为 lo - s <= x_k <= hi + s，代价加 W_LIN·s + ½·W_QUAD·s²
    //   W_LIN  越大越接近硬约束（超过约束的对偶乘子时，能满足就一定满足）
    //   W_QUAD 越大，越界后往回拉得越狠
    //   s 的单位与被约束状态相同（摆角/俯仰 rad，角速度 rad/s）
    constexpr double SOFT_W_LIN_THETA   = 1.0e4;   // θ_ll / θ_lr
    constexpr double SOFT_W_QUAD_THETA  = 1.0e7;
    constexpr double SOFT_W_LIN_PITCH   = 1.0e4;   // θ_b
    constexpr double SOFT_W_QUAD_PITCH  = 1.0e7;
    constexpr double SOFT_W_LIN_DTHETA  = 1.0e3;   // dθ_ll / dθ_lr
    constexpr double SOFT_W_QUAD_DTHETA = 1.0e5;

    // ---- 6) 状态权重 Q 的对角线 [s, ds, phi, dphi, th_ll, dth_ll, th_lr, dth_lr, th_b, dth_b] ----
    //   普通模式
    constexpr double Q_NORMAL[10] = {6000, 90, 1, 1, 9000, 300, 9000, 300, 600000, 500}; //   普通模式
 
    constexpr double Q_SPIN[10]   = {1, 90, 1, 1500, 15400, 270, 15400, 270, 5.4e6, 980}; //   小陀螺模式
}

class Robot_MPC
{
public:
    static constexpr int n  = 10;   // 状态个数
    static constexpr int p  = 4;    // 控制个数
    static constexpr int NP = 15;   // 预测步数：15 * 8ms = 120ms

    // ===== 状态约束（软约束）：{状态下标, 腿长来源, 下限, 上限, 松弛线性权, 松弛二次权} =====
    //   leg = -1 → 用固定 lo/hi
    //   leg =  0 → 用左腿长算摆角限幅   leg = 1 → 用右腿长
    struct ConState { int idx; int leg; double lo, hi; double w_lin, w_quad; };
    static constexpr int NCON = 5;
    static constexpr ConState CON[NCON] = {
        {4,  0, 0.0, 0.0,
         MPC_CFG::SOFT_W_LIN_THETA,  MPC_CFG::SOFT_W_QUAD_THETA},                    // θ_ll：随左腿长
        {6,  1, 0.0, 0.0,
         MPC_CFG::SOFT_W_LIN_THETA,  MPC_CFG::SOFT_W_QUAD_THETA},                    // θ_lr：随右腿长
        {8, -1, -MPC_CFG::PITCH_MAX, MPC_CFG::PITCH_MAX,
         MPC_CFG::SOFT_W_LIN_PITCH,  MPC_CFG::SOFT_W_QUAD_PITCH},                    // θ_b ：固定
        {5, -1, -MPC_CFG::DTHETA_MAX, MPC_CFG::DTHETA_MAX,
         MPC_CFG::SOFT_W_LIN_DTHETA, MPC_CFG::SOFT_W_QUAD_DTHETA},                   // dθ_ll
        {7, -1, -MPC_CFG::DTHETA_MAX, MPC_CFG::DTHETA_MAX,
         MPC_CFG::SOFT_W_LIN_DTHETA, MPC_CFG::SOFT_W_QUAD_DTHETA},                   // dθ_lr
    };

    // ===== QP 决策变量：[U(NP*p) ; s(NCON)]，s 为每个状态约束的松弛量 =====
    static constexpr int NU_QP = NP * p;            // 60：控制序列
    static constexpr int NV_QP = NU_QP + NCON;      // 65：控制 + 松弛

    /// 腿长 -> 摆角限幅（线性插值）
    static double ThetaLimit(double L)
    {
        const double t = std::clamp(
            (L - MPC_CFG::LEG_MIN) / (MPC_CFG::LEG_MAX - MPC_CFG::LEG_MIN), 0.0, 1.0);
        return MPC_CFG::THETA_AT_MIN + t * (MPC_CFG::THETA_AT_MAX - MPC_CFG::THETA_AT_MIN);
    }

    // ===== 各类约束占用的行数（Init 里用来算 nC）=====
    static constexpr int NROW_DU    = 2 * NP * p;                     // Δu
    static constexpr int NROW_CON   = 2 * NP * NCON;                  // 绝对状态
    static constexpr int NROW_ALL   = NROW_DU + NROW_CON;

    using VecX = Eigen::Matrix<double, n, 1>;
    using VecU = Eigen::Matrix<double, p, 1>;

    void Init(double dt);

    /**
     * @brief 切换普通 / 小陀螺权重。只能和 Run() 在同一个线程调用。
     *        模式变化后，下一次 Run() 会换 Q 并冷启动重算 Riccati（上电外唯一一次几 ms 的重建）。
     */
    void SetSpinMode(bool spin)
    {
        if (spin != spin_mode) { spin_mode = spin; mode_dirty = true; }
    }

    bool Run(const VecX& err, double L_l, double L_r, VecU& u);

    /**
     * @brief  QP 失败时的兜底：u = -K·e，再等比缩放到与 MPC 相同的力矩限幅。
     *         K 来自当前 Q/R/腿长下的 Riccati；不含 Δu 与状态约束。
     *         只有 RebuildOK() 为 true 时 K 才可信。
     */
    VecU Fallback(const VecX& err) const
    {
        VecU u = -K * err;
        double sc = 1.0;
        for (int i = 0; i < p; ++i) {
            const double hi = (u(i) >= 0.0) ? u_max(i) : -u_min(i);
            if (std::abs(u(i)) > 1e-9) sc = std::min(sc, hi / std::abs(u(i)));
        }
        return u * sc;
    }

    /**
     * @brief 设置 Δu 约束的起点（上一拍实际生效的通道命令）
     */
    void SetLastControl(const VecU& u)
    {
        for (int i = 0; i < p; ++i)
            u_last(i) = std::clamp(u(i), u_min(i), u_max(i));
    }

    int Iterations() const { return wsr; }
    bool RebuildOK() const { return rebuild_ok; }   // 重建是否成功
    bool RiccatiOK() const { return riccati_ok; }   // Riccati 是否发散
    double StepDelta(int i) const { return u_step(i); }   // 本拍 u0 − 上一拍 [Nm]
    double Slack(int c) const { return U(NU_QP + c); }    // 第 c 个状态约束的松弛量（>0 表示预测会越界）

private:
    void Rebuild(double L_l, double L_r, bool cold);  // 重建模型 + H（cold=冷启动 DARE）
    void BuildAB(double L_l, double L_r);   // 腿长 -> Ac, Bc（5x5 精确）
    void BuildGradient();                   // 每周期算 g
    void SetBounds();                       // 每周期更新上下限
    void Fill_Constraint_Am();              // 状态约束的矩阵行（随 Cp 变，Rebuild 里调）
    void Fill_Constraint_Bounds();          // 状态约束的上下界（随 e0 变，每周期调）

    double dt = MPC_CFG::DT, age = 0.0;   // mpc控制周期，强制重建时间

    Eigen::MatrixXd Ac, Bc;                 // 连续模型
    Eigen::MatrixXd Ad, Bd;                 // 离散模型
    Eigen::MatrixXd Mp, Cp;                 // 预测矩阵
    Eigen::MatrixXd Qb, Rb;                 // 权重
    Eigen::MatrixXd CtQ, H;                 // 预存项 / Hessian
    Eigen::MatrixXd F, K, Q, R;             // 终端代价 / 增益 / 权重
    Eigen::VectorXd g, Xfree, U;            // 斜率 / 自由响应 / 解（g、U 含松弛量，长 NV_QP）
    Eigen::VectorXd u_step;                 // 本拍 u0 − 上一拍 [Nm]
    Eigen::VectorXd lb, ub, lbA, ubA;       // 上下限
    Eigen::VectorXd u_min, u_max, du;       // 力矩限幅 / 变化率限幅

    // qpOASES 的约束矩阵按「行优先」存储，Eigen 默认列优先，必须显式声明
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Am;

    VecX e0;                                // 当前误差态
    VecU u_last;                            // 上一周期下发值
    Eigen::Vector2d Lnow{0.18, 0.18}, Lold{0.0, 0.0};

    std::unique_ptr<qpOASES::SQProblem> qp;  // H 会变 -> SQProblem
    bool ready = false, solver_inited = false, ok = false;
    bool spin_mode = false, mode_dirty = false;   // 当前权重模式 / 需要换 Q 并冷启动重建
    bool rebuild_ok = false;   // 最近一次 Rebuild 的数据是否有效（有限 + Riccati 未发散）
    bool riccati_ok = false;   // Dare 是否正常返回
    int  wsr = 0; // QP 迭代计数器
};

#endif // ROBOT_MPC_H
