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
  *  每周期（4 ms）:
  *      g = CtQ · Mp · e    ──>   QP   ──>   U(60)   ──>   只取前 4 个
  *
  *  腿长变化 >2 mm 或 100 ms 一次:
  *      5x5 求 A_c,B_c -> 离散 A_d,B_d -> Riccati 得 F(=P),K
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

class Robot_MPC
{
public:
    static constexpr int n  = 10;   // 状态个数
    static constexpr int p  = 4;    // 控制个数
    static constexpr int NP = 15;   // 预测步数：15 * 4ms = 60ms

    using VecX = Eigen::Matrix<double, n, 1>;
    using VecU = Eigen::Matrix<double, p, 1>;

    void Init(double dt);

    bool Run(const VecX& err, double L_l, double L_r, VecU& u);

    /**
     * @brief LQR 兜底：u = -K·e，再等比缩放到与 MPC 相同的力矩限幅
     *        等比缩放（而不是逐元素截断）可以保住力矩合成方向
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
     * @attention 必须夹进盒约束内：LQR 的髋矩按 Fitting_K·err 算，err[8] 上限 1.0、
     *            K≈-128，能算出 ±100 Nm。若 u_last 超出 [u_min, u_max]，
     *            k=0 的 Δu 约束与盒约束会互斥，QP 直接 infeasible。
     */
    void SetLastControl(const VecU& u)
    {
        for (int i = 0; i < p; ++i)
            u_last(i) = std::clamp(u(i), u_min(i), u_max(i));
    }

    int Iterations() const { return wsr; }

private:
    void Rebuild(double L_l, double L_r, bool cold);  // 重建模型 + H（cold=冷启动 DARE）
    void BuildAB(double L_l, double L_r);   // 腿长 -> Ac, Bc（5x5 精确）
    void BuildGradient();                   // 每周期算 g
    void SetBounds();                       // 每周期更新上下限

    double dt = 0.004, age = 0.0;   // mpc控制周期，强制重建时间

    Eigen::MatrixXd Ac, Bc;                 // 连续模型
    Eigen::MatrixXd Ad, Bd;                 // 离散模型
    Eigen::MatrixXd Mp, Cp;                 // 预测矩阵
    Eigen::MatrixXd Qb, Rb;                 // 权重
    Eigen::MatrixXd CtQ, H;                 // 预存项 / Hessian
    Eigen::MatrixXd F, K, Q, R;             // 终端代价 / 增益 / 权重
    Eigen::VectorXd g, Xfree, U;            // 斜率 / 自由响应 / 解
    Eigen::VectorXd lb, ub, lbA, ubA;       // 上下限
    Eigen::VectorXd u_min, u_max, du;       // 力矩限幅 / 变化率限幅

    // qpOASES 的约束矩阵按「行优先」存储，Eigen 默认列优先，必须显式声明
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Am;

    VecX e0;                                // 当前误差态
    VecU u_last;                            // 上一周期下发值
    Eigen::Vector2d Lnow{0.18, 0.18}, Lold{0.0, 0.0};

    std::unique_ptr<qpOASES::SQProblem> qp;  // H 会变 -> SQProblem
    bool ready = false, solver_inited = false, ok = false;
    int  wsr = 0;
};

#endif // ROBOT_MPC_H
