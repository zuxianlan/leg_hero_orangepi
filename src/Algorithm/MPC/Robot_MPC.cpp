/**
  ******************************************************************************
  * @file           : Robot_MPC.h
  * @author         : Zhu_hechen
  * @brief          : 机体全状态线性MPC控制
  * @attention      : None
  * @date           : 2026/10/27
  ******************************************************************************
  *  实现思路（按顺序读）：
  *    Init()     : 分配矩阵、设 Q/R/限幅、建一次模型
  *    BuildAB()  : 腿长 -> 5x5 牛顿方程 -> Ac(10x10), Bc(10x4)
  *    Rebuild()  : Ac,Bc -> Ad,Bd -> F(=P),K -> Mp,Cp -> H -> qp.init
  *    Run()      : 每周期：判重建 -> g -> 边界 -> QP -> 取 U 前 4 个
  ******************************************************************************
  */
#include "Robot_MPC.h"
#include <cmath>
#include <algorithm>

namespace {

// ===== 机器人物理参数（与 MATLAB 脚本 Step 2 一致）=====
constexpr double Rw = 0.06;        // 驱动轮半径            (m)
constexpr double Rl = 0.33;        // 两轮间距的一半        (m)
constexpr double lc = 0.037;       // 机体质心到腿部关节距离 (m)
constexpr double mw = 0.26;        // 单个驱动轮质量        (kg)
constexpr double ml = 2.6183599;   // 单条腿质量            (kg)
constexpr double mb = 35.542;      // 机体质量              (kg)
constexpr double Iw = 7.51e-7;     // 驱动轮转动惯量        (kg·m²)
constexpr double Ib = 0.960;       // 机体俯仰转动惯量      (kg·m²)
constexpr double Iz = 0.926;       // 整车 yaw 转动惯量     (kg·m²)
constexpr double kGrav = 9.81;     // 重力加速度（改名避免与成员 g 冲突）

// ===== 腿长 -> (l_w, l_b, I_l)，整表抄自 MATLAB 的 Leg_data_l =====
// 第1列: 腿长 L；第2列: l_w；第3列: l_b；第4列: I_l
constexpr double LEG[24][4] = {
    {0.10722,0.11251,0.06863,0.03200000}, {0.11190,0.11367,0.06996,0.03386957},
    {0.12190,0.11631,0.07292,0.03573913}, {0.13190,0.11914,0.07605,0.03760870},
    {0.14190,0.12214,0.07930,0.03947826}, {0.15190,0.12529,0.08267,0.04134783},
    {0.16190,0.12859,0.08614,0.04321739}, {0.17190,0.13202,0.08968,0.04508696},
    {0.18190,0.13556,0.09331,0.04695652}, {0.19190,0.13921,0.09699,0.04882609},
    {0.20190,0.14296,0.10073,0.05069565}, {0.21190,0.14680,0.10452,0.05256522},
    {0.22190,0.15072,0.10836,0.05443478}, {0.23190,0.15472,0.11223,0.05630435},
    {0.24190,0.15878,0.11614,0.05817391}, {0.25190,0.16291,0.12008,0.06004348},
    {0.26190,0.16710,0.12405,0.06191304}, {0.27190,0.17134,0.12805,0.06378261},
    {0.28190,0.17563,0.13207,0.06565217}, {0.29190,0.17996,0.13611,0.06752174},
    {0.30190,0.18434,0.14017,0.06939130}, {0.31190,0.18876,0.14425,0.07126087},
    {0.32190,0.19322,0.14834,0.07313043}, {0.33190,0.19771,0.15246,0.07500000},
};

// 线性插值：给腿长 L，输出 o = {l_w, l_b, I_l}
void LegParams(double L, double* o)
{
    constexpr int M = 24;
    int i = 0;
    if      (L <= LEG[0][0])   i = 0;                       // 低于表范围：取第一点
    else if (L >= LEG[M-1][0]) i = M - 2;                   // 高于表范围：取最后一段
    else while (i < M-2 && LEG[i+1][0] < L) ++i;            // 找到 L 所在区间

    const double t = std::clamp((L - LEG[i][0]) / (LEG[i+1][0] - LEG[i][0]), 0.0, 1.0);
    for (int k = 0; k < 3; ++k)
        o[k] = LEG[i][k+1] + t * (LEG[i+1][k+1] - LEG[i][k+1]);
}

// ===== 离散 Riccati 迭代 =====
// 反复用 P <- Q + Acl'·P·Acl + K'·R·K 更新，直到 P 不再变化。
// 收敛后的 P 就是"从当前状态走到无穷远的代价矩阵"，同时得到 LQR 增益 K。
// P 用上一周期的结果热启动（腿长变化很慢），通常几次迭代就收敛。
void Dare(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
          const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
          Eigen::MatrixXd& P, Eigen::MatrixXd& K, int maxit)
{
    if (!P.allFinite() || P.rows() != A.rows()) P = Q;

    for (int it = 0; it < maxit; ++it) {
        const Eigen::MatrixXd BtP = B.transpose() * P;
        const Eigen::MatrixXd Kk  = (R + BtP * B).ldlt().solve(BtP * A);   // 当前增益
        const Eigen::MatrixXd Acl = A - B * Kk;                            // 闭环矩阵
        const Eigen::MatrixXd Pn  = Q + Acl.transpose() * P * Acl
                                      + Kk.transpose() * R * Kk;           // 更新 P

        if (!Pn.allFinite()) { P = Q; K.setZero(); return; }               // 数值发散保护

        const double e = (Pn - P).cwiseAbs().maxCoeff();
        const double s = 1.0 + P.cwiseAbs().maxCoeff();
        P = Pn; K = Kk;
        if (e < 1e-9 * s) return;                                          // 收敛
    }
}

} // namespace

/**
 * @brief  上电初始化：分配全部矩阵、设置 Q/R/限幅/状态约束、建一次模型并冷启动 DARE
 * @param  dt_  离散步长 [s]，必须等于 4 ms 任务的实际周期（当前 0.004）
 * @return 无
 */
void Robot_MPC::Init(double dt_)
{
    dt = dt_;
    const int nX = (NP + 1) * n;   // 160：未来 0..15 步的状态
    const int nU = NP * p;         // 60 ：未来 15 步的控制
    const int nC = NROW_ALL;                 // 行数由头文件 MPC_CFG 自动决定

    Ac = Eigen::MatrixXd::Zero(n, n);
    Bc = Eigen::MatrixXd::Zero(n, p);
    Ad = Eigen::MatrixXd::Identity(n, n);
    Bd = Eigen::MatrixXd::Zero(n, p);
    Mp = Eigen::MatrixXd::Zero(nX, n);
    Cp = Eigen::MatrixXd::Zero(nX, nU);
    Qb = Eigen::MatrixXd::Zero(nX, nX);
    Rb = Eigen::MatrixXd::Zero(nU, nU);
    CtQ = Eigen::MatrixXd::Zero(nU, nX);
    H  = Eigen::MatrixXd::Zero(nU, nU);

    g = Eigen::VectorXd::Zero(nU);
    Xfree = Eigen::VectorXd::Zero(nX);
    lb = ub = U = Eigen::VectorXd::Zero(nU);
    lbA = ubA = Eigen::VectorXd::Zero(nC);
    Am.setZero(nC, nU);

    // 权重：直接沿用 LQR 的 Q、R（顺序见头文件）
    Q = Eigen::MatrixXd::Zero(n, n);
    Q.diagonal() << 6000, 90, 30000, 1500, 19000, 300, 19000, 300, 600000, 500;
    R = Eigen::MatrixXd::Zero(p, p);
    R.diagonal() << 200, 200, 8, 8;

    F = Q;                                  // 占位，Rebuild 里会被 Riccati 的 P 覆盖
    K = Eigen::MatrixXd::Zero(p, n);

    // 力矩限幅与变化率限幅（★上线前按实测峰值修改★）
    u_min.resize(p); u_min << -15.0, -15.0, -45.0, -45.0;   // [T_wl, T_wr, T_bl, T_br]
    u_max.resize(p); u_max <<  15.0,  15.0,  45.0,  45.0;
    // du = 每个预测步(4ms)允许的力矩变化量，即速率限制：
    du.resize(p);    du    <<   2.0,   2.0,   8.0,   8.0;   // Nm / 每步(4ms)

    e0.setZero();
    u_last.setZero();
    u_step = Eigen::VectorXd::Zero(p);

    qp = std::make_unique<qpOASES::SQProblem>(nU, nC);
    qpOASES::Options o;
    o.printLevel = qpOASES::PL_NONE;        // 实机不要打印，会卡实时
    qp->setOptions(o);

    Rebuild(0.18, 0.18, true);               // 冷启动：把 P 迭代到收敛
}

/**
 * @brief  由左右腿长构建连续模型 Ac、Bc
 *         原理：5 个牛顿方程写成 M·q̈ = E·u + G·θ，平衡点处 ∂q̈/∂θ = M⁻¹G、∂q̈/∂u = M⁻¹E，
 *         与 MATLAB 里 jacobian 的结果完全等价，不需要符号推导
 * @param  Ll  左腿长 [m]
 * @param  Lr  右腿长 [m]
 * @return 无
 */
void Robot_MPC::BuildAB(double Ll, double Lr)
{
    double pl[3], pr[3];
    LegParams(Ll, pl);
    LegParams(Lr, pr);
    const double lwl = pl[0], lbl = pl[1], Ill = pl[2];   // 左腿
    const double lwr = pr[0], lbr = pr[1], Ilr = pr[2];   // 右腿

    Eigen::Matrix<double, 5, 5> M  = Eigen::Matrix<double, 5, 5>::Zero();
    Eigen::Matrix<double, 5, 7> GE = Eigen::Matrix<double, 5, 7>::Zero();  // 前3列G，后4列E

    M(0,0) = Iw*Ll/Rw + mw*Rw*Ll + ml*Rw*lbl;
    M(0,2) = ml*lwl*lbl - Ill;
    M(1,1) = Iw*Lr/Rw + mw*Rw*Lr + ml*Rw*lbr;
    M(1,3) = ml*lwr*lbr - Ilr;
    const double c3 = mw*Rw*Rw + Iw + ml*Rw*Rw + mb*Rw*Rw/2;
    M(2,0) = M(2,1) = -c3;
    M(2,2) = -(ml*Rw*lwl + mb*Rw*Ll/2);
    M(2,3) = -(ml*Rw*lwr + mb*Rw*Lr/2);
    const double e1 = mw*Rw*lc + Iw*lc/Rw + ml*Rw*lc;
    M(3,0) = M(3,1) = e1;
    M(3,2) = ml*lwl*lc;
    M(3,3) = ml*lwr*lc;
    M(3,4) = -Ib;
    const double g1 = Iz*Rw/(2*Rl) + Iw*Rl/Rw;
    M(4,0) = g1;  M(4,1) = -g1;
    M(4,2) = Iz*Ll/(2*Rl);
    M(4,3) = -Iz*Lr/(2*Rl);

    GE(0,0) = -(ml*lwl + mb*Ll/2) * kGrav;
    GE(1,1) = -(ml*lwr + mb*Lr/2) * kGrav;
    GE(3,2) = -mb*kGrav*lc;
    GE(0,3) = 1 + Ll/Rw;  GE(0,5) = -1;
    GE(1,4) = 1 + Lr/Rw;  GE(1,6) = -1;
    GE(2,3) = -1;         GE(2,4) = -1;
    GE(3,3) = lc/Rw;  GE(3,4) = lc/Rw;  GE(3,5) = 1;  GE(3,6) = 1;
    GE(4,3) = Rl/Rw;  GE(4,4) = -Rl/Rw;

    const Eigen::Matrix<double, 5, 7> Qq = M.partialPivLu().solve(GE);
    Ac.setZero();
    Bc.setZero();
    Ac(0,1) = Ac(2,3) = Ac(4,5) = Ac(6,7) = Ac(8,9) = 1.0;   // 位置导数 = 速度

    const int col[3] = {4, 6, 8};                             // θ_ll, θ_lr, θ_b 在状态中的位置
    for (int c = 0; c < 3; ++c) {
        const int j = col[c];
        Ac(1,j) = Rw*(Qq(0,c) + Qq(1,c)) / 2;                             // s̈
        Ac(3,j) = Rw*(Qq(1,c) - Qq(0,c)) / (2*Rl)
                - Ll*Qq(2,c)/(2*Rl) + Lr*Qq(3,c)/(2*Rl);                  // φ̈
        Ac(5,j) = Qq(2,c);
        Ac(7,j) = Qq(3,c);
        Ac(9,j) = Qq(4,c);
    }
    for (int k = 0; k < p; ++k) {
        const int c = 3 + k;
        Bc(1,k) = Rw*(Qq(0,c) + Qq(1,c)) / 2;
        Bc(3,k) = Rw*(Qq(1,c) - Qq(0,c)) / (2*Rl)
                - Ll*Qq(2,c)/(2*Rl) + Lr*Qq(3,c)/(2*Rl);
        Bc(5,k) = Qq(2,c);
        Bc(7,k) = Qq(3,c);
        Bc(9,k) = Qq(4,c);
    }

    // ===== 力矩方向对齐 =====
    // 现 LQR 用的是 T = Fitting_K·e，而 icare 的 K 对应 u = -Kx，
    // 说明电机力矩正方向与模型里 T 的正方向相反。
    // 这里把 B 取负，让 QP 的解 u 直接就是「发给电机的力矩」，
    // 从而 MPC 的无约束输出与现 LQR 同号（用这个开关可以关掉）。
    constexpr bool kHwTorqueFlipped = true;
    if (kHwTorqueFlipped) Bc = -Bc;
}

/**
 * @brief  重建模型 + 代价矩阵：离散化 → DARE(F=P,K) → condensing → H → 约束矩阵
 *         只在腿长变化 >2 mm 或 100 ms 未更新时执行（约 0.3~0.8 ms）
 * @param  Ll    左腿长 [m]
 * @param  Lr    右腿长 [m]
 * @param  cold  true  = 冷启动 DARE（3000 次，约 5.5 ms，仅上电一次）
 *               false = 热启动跟踪（40 次，约 75 μs）
 * @return 无
 */
void Robot_MPC::Rebuild(double Ll, double Lr, bool cold)
{
    BuildAB(Ll, Lr);                                        // ① 腿长 -> Ac,Bc

    const Eigen::MatrixXd Adt = Ac * dt;
    Ad = Eigen::MatrixXd::Identity(n, n) + Adt + 0.5 * Adt * Adt;
    Bd = (Eigen::MatrixXd::Identity(n, n) * dt + 0.5 * Adt * dt) * Bc;

    // ③ Riccati -> F(=P) 终端代价，同时得到兜底增益 K
    Dare(Ad, Bd, Q, R, F, K, cold ? 3000 : 40);   // 冷启动收敛 / 热启动跟踪

    Mp.setZero();
    Cp.setZero();
    Eigen::MatrixXd Ap = Eigen::MatrixXd::Identity(n, n);
    Mp.topRows(n) = Ap;
    for (int i = 1; i <= NP; ++i) { Ap = Ad * Ap; Mp.middleRows(i*n, n) = Ap; }
    for (int i = 1; i <= NP; ++i) {
        Eigen::MatrixXd Aq = Eigen::MatrixXd::Identity(n, n);
        for (int j = i - 1; j >= 0; --j) {
            Cp.block(i*n, j*p, n, p) = Aq * Bd;
            Aq = Ad * Aq;
        }
    }

    Qb.setZero();
    Rb.setZero();
    for (int k = 0; k < NP; ++k) {
        Qb.block(k*n, k*n, n, n) = Q;
        Rb.block(k*p, k*p, p, p) = R;
    }
    Qb.block(NP*n, NP*n, n, n) = F;

    CtQ = Cp.transpose();                                  // 60 x 160
    for (int k = 0; k < NP; ++k)
        for (int i = 0; i < n; ++i)
            CtQ.col(k*n + i) *= Q(i,i);
    CtQ.middleCols(NP*n, n) = Cp.middleRows(NP*n, n).transpose() * F;
    H = CtQ * Cp + Rb;

    // ⑦ Δu 约束矩阵（常数）：|u_k - u_{k-1}| <= du
    Am.setZero();
    for (int k = 0; k < NP; ++k) for (int j = 0; j < p; ++j) {
        const int c = k*p + j, r = 2*c;
        Am(r,   c) =  1.0;
        Am(r+1, c) = -1.0;
        if (k) { Am(r, c-p) = -1.0; Am(r+1, c-p) = 1.0; }
    }

    // ⑦b 状态约束矩阵（随 Cp 变化）：θ_ll / θ_lr / θ_b 的上下界
    Fill_Constraint_Am();

    BuildGradient();
    SetBounds();

    Lold << Ll, Lr;
    age = 0.0;
    ready = true;
}

/**
 * @brief  每周期计算线性项 g = CtQ · Mp · e0（参考已折算进误差态，故 Xref = 0）
 * @return 无
 */
void Robot_MPC::BuildGradient()
{
    Xfree.noalias() = Mp * e0;        // 不出力时未来 16 步的状态
    g.noalias()     = CtQ * Xfree;    // 代价对 U 的斜率
}

/**
 * @brief  每周期更新 QP 边界：力矩盒约束、Δu 约束、状态约束右端项
 * @return 无
 */
void Robot_MPC::SetBounds()
{
    // 力矩硬限幅
    for (int k = 0; k < NP; ++k) {
        lb.segment(k*p, p) = u_min;
        ub.segment(k*p, p) = u_max;
    }

    // Δu 限幅：先全部放开下界，只填上界
    lbA.setConstant(-1e20);
    ubA.setConstant( 1e20);
    for (int k = 0; k < NP; ++k) for (int j = 0; j < p; ++j) {
        const int r = 2 * (k*p + j);
        const double d = du(j);
        if (k == 0) {                       // 第一步要和上一周期实际下发值比
            ubA(r)   = d + u_last(j);
            ubA(r+1) = d - u_last(j);
        } else {
            ubA(r) = ubA(r+1) = d;
        }
    }

    // 状态约束的右端项（随当前 e0 变化）
    Fill_Constraint_Bounds();
}

/**
 * @brief  填充状态约束的矩阵行：每个受约束状态、每个预测步各写「上界/下界」两行
 * @return 无
 * @attention 行内系数取自 Cp，所以只能在 Rebuild 里调用（Cp 变化时必须重填）
 */
void Robot_MPC::Fill_Constraint_Am()
{
    int r = NROW_DU;                     // 前 NROW_DU 行是 Δu，已在 Rebuild() 里填好

    // ---- ① 绝对状态约束（θ 随腿长、θ_b 固定）----
    for (int c = 0; c < NCON; ++c)
        for (int k = 1; k <= NP; ++k) {
            const int src = k * n + CON[c].idx;
            Am.row(r)     =  Cp.row(src);
            Am.row(r + 1) = -Cp.row(src);
            r += 2;
        }

    // ---- ② 绝对速度约束 |v_k| <= VEL_ABS_MAX ----
    if constexpr (MPC_CFG::VEL_ABS_ON) {
        for (int k = 1; k <= NP; ++k) {
            const int src = k * n + 1;                 // 状态 1 = ds
            Am.row(r)     =  Cp.row(src);
            Am.row(r + 1) = -Cp.row(src);
            r += 2;
        }
    }

    // ---- ③ 加速度约束 |v_k - v_{k-1}| <= ACC_MAX·dt ----
    if constexpr (MPC_CFG::VEL_RATE_ON) {
        for (int k = 1; k <= NP; ++k) {
            Am.row(r)     =  Cp.row(k*n + 1) - Cp.row((k-1)*n + 1);
            Am.row(r + 1) = -Am.row(r);
            r += 2;
        }
    }
}

/**
 * @brief  填充所有约束的右端项（每周期调用，随 e0 / 腿长 / v_ref 变化）
 *         第 k 步状态 x_k = Xfree_k + Cp_k·U，约束 lo ≤ x_k ≤ hi 化作
 *             Cp_k·U ≤  hi − m
 *            −Cp_k·U ≤ −lo + m
 * @return 无
 */
void Robot_MPC::Fill_Constraint_Bounds()
{
    int r = NROW_DU;

    // ---- ① 绝对状态约束 ----
    for (int c = 0; c < NCON; ++c) {
        double lo = CON[c].lo, hi = CON[c].hi;
        if (CON[c].leg >= 0) {                     // 摆角：按对应腿长算限幅
            const double lim = ThetaLimit(Lnow(CON[c].leg));
            lo = -lim;
            hi =  lim;
        }
        const double x0 = e0(CON[c].idx);
        lo = std::min(lo, x0 - MPC_CFG::CON_RELAX);   // 已越界时让开，保证可行
        hi = std::max(hi, x0 + MPC_CFG::CON_RELAX);

        for (int k = 1; k <= NP; ++k) {
            const double m = Xfree(k*n + CON[c].idx);
            ubA(r)     =  hi - m;
            ubA(r + 1) = -lo + m;
            r += 2;
        }
    }

    // ---- ② 绝对速度约束：v = e1 + vref ----
    if constexpr (MPC_CFG::VEL_ABS_ON) {
        const double margin = MPC_CFG::ACC_MAX * dt;   // 一步最多能变多少
        const double x1 = e0(1);
        const double lo = std::min(-MPC_CFG::VEL_ABS_MAX - vref, x1 - margin);
        const double hi = std::max( MPC_CFG::VEL_ABS_MAX - vref, x1 + margin);
        for (int k = 1; k <= NP; ++k) {
            const double m = Xfree(k*n + 1);
            ubA(r)     =  hi - m;
            ubA(r + 1) = -lo + m;
            r += 2;
        }
    }

    // ---- ③ 加速度约束（vref 在相邻两步相减时抵消）----
    if constexpr (MPC_CFG::VEL_RATE_ON) {
        const double dv = MPC_CFG::ACC_MAX * dt;       // 每步允许的速度变化
        for (int k = 1; k <= NP; ++k) {
            const double m = Xfree(k*n + 1) - Xfree((k-1)*n + 1);
            ubA(r)     =  dv - m;
            ubA(r + 1) =  dv + m;
            r += 2;
        }
    }
}

/**
 * @brief  MPC 主流程：判重建 → 算 g → 更新边界 → 求解 QP → 取第一步 4 个力矩
 * @param  err  当前误差态 e(10) = x - x_ref
 * @param  Ll   左腿长 [m]
 * @param  Lr   右腿长 [m]
 * @param  u    输出：第一步的 [T_wl, T_wr, T_bl, T_br]，单位 Nm
 * @return true = 求解成功，u 有效；false = 失败，调用方应改用 Fallback()
 */
bool Robot_MPC::Run(const VecX& err, double Ll, double Lr, VecU& u, double v_ref)
{
    if (!err.allFinite()) { ok = false; return false; }          // ① 数据检查

    e0 = err;                                                    // ② 存当前误差态
    vref = v_ref;                                                //    参考速度（绝对速度约束用）
    Lnow << std::clamp(Ll, 0.107, 0.332), std::clamp(Lr, 0.107, 0.332);

    age += dt;
    // ③ 腿长变化 >2mm 或超过 100ms -> 重建
    if (!ready || (Lnow - Lold).cwiseAbs().maxCoeff() > 2e-3 || age > 0.1)
        Rebuild(Lnow(0), Lnow(1), false);     // 热启动：只跟踪几十次

    BuildGradient();                                             // ④ 算 g
    SetBounds();                                                 // ⑤ 更新边界

    // ⑥ 求解：首次冷启动 init，之后一律 hotstart（H 变了 SQProblem 会自动适配）
    wsr = 600;      //求解器单次求解允许的最大迭代步数
    qpOASES::returnValue st;
    if (!solver_inited) {
        st = qp->init(H.data(), g.data(), Am.data(), lb.data(), ub.data(),
                      lbA.data(), ubA.data(), wsr);
        solver_inited = (st == qpOASES::SUCCESSFUL_RETURN);
    } else {
        st = qp->hotstart(H.data(), g.data(), Am.data(), lb.data(), ub.data(),
                          lbA.data(), ubA.data(), wsr);
        if (st != qpOASES::SUCCESSFUL_RETURN) {                   // 热启动失败 -> 冷启动重来
            wsr = 600;
            st = qp->init(H.data(), g.data(), Am.data(), lb.data(), ub.data(),
                          lbA.data(), ubA.data(), wsr);
        }
    }
    ok = (st == qpOASES::SUCCESSFUL_RETURN);
    if (!ok) return false;

    qp->getPrimalSolution(U.data());                             // ⑦ 取解
    u = U.head(p);                                               //    只用第一步的 4 个力矩
    for (int i = 0; i < p; ++i) u_step(i) = u(i) - u_last(i);

    return true;                                                 // age 已在第 ③ 步之前累加
}
