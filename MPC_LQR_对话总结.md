# leg_hero_27：MPC / LQR / 卡尔曼 讨论总结

> 整理时间：2026-10-25  
> 适用代码：`/home/zhc/code/hero_orange_pi/leg_hero_27`  
> 说明：本文是对此前对话的整理，不是官方设计文档；具体行为以当前仓库代码为准。

---

## 目录

1. [核心结论](#1-核心结论)
2. [线性 MPC 的结构](#2-线性-mpc-的结构)
3. [MPC 的约束现状](#3-mpc-的约束现状)
4. [dt 不匹配问题](#4-dt-不匹配问题)
5. [卡尔曼滤波、INS 与打滑策略](#5-卡尔曼滤波ins-与打滑策略)
6. [检录模式与原地掉头](#6-检录模式与原地掉头)
7. [QP 求解器与嵌入式部署](#7-qp-求解器与嵌入式部署)
8. [线性 MPC、SQP NMPC、CasADi NMPC 对比](#8-线性-mpcsqp-nmpccasadi-nmpc-对比)
9. [LQR 与 MPC 的关系](#9-lqr-与-mpc-的关系)
10. [终端代价、终端集、热启动](#10-终端代价终端集热启动)
11. [MATLAB LQR 脚本分析](#11-matlab-lqr-脚本分析)
12. [关键参数汇总](#12-关键参数汇总)
13. [风险与待办](#13-风险与待办)
14. [关键文件索引](#14-关键文件索引)
15. [10 状态全状态线性 MPC（2026-10-27 落地）](#15-10-状态全状态线性-mpc2026-10-27-落地)
16. [复核与修复（2026-10-27）](#16-复核与修复2026-10-27)

---

## 1. 核心结论

- 当前实际使用的线性 MPC 是 **4 状态、2 控制、N=15** 的 condensed QP。
- 状态：`[roll, roll_rate, height, height_rate]`
- 控制：`[F_left, F_right]`
- QP 决策变量是未来 15 步控制序列：`U = [u0; ...; u14]`，维度 `15*2 = 30`。
- 求解器：`qpOASES::QProblem`（active-set），第一次 `init`，之后 `hotstart`。
- 当前 MPC **只有输入上下界 ±150**；图片里的 `20~150 N`、`|ΔF| <= 30 N/step`、`G U <= h` 都还没有实现。
- 当前实际控制周期是 **1 ms**，但 `dt_mpc = 0.002 s`，存在模型步长与执行周期不匹配。
- 底盘 2 状态卡尔曼的数值逻辑和旧版基本一致，只是封装成了 `KF_Init / Set_Mode / Set_Z`。
- 旧版 INS 里的四元数 EKF/Tactical Fusion 已不再参与运行；姿态直接使用 mc02 上发数据。
- LQR 与 MPC 不是重复控制：LQR 主要管轮矩/髋矩/平衡/速度/yaw，MPC 主要管腿力/高度/roll。
- `Chassis.cpp` 里的 `float P[40][6]` 不是 Riccati 解，而是 LQR 增益的拟合系数表。
- MATLAB 脚本里的 Riccati 解 `P` 被计算出来后丢弃，最终只使用增益 `K` 的拟合结果。

---

## 2. 线性 MPC 的结构

### 2.1 模型

离散线性模型：

```text
x_{k+1} = A * x_k + B * u_k
```

状态顺序：

```text
x(0) = roll
x(1) = roll_rate
x(2) = height
x(3) = height_rate
```

控制顺序：

```text
u(0) = F_left
u(1) = F_right
```

`A/B` 在 `MPC_Init()` 中按 `dt_mpc` 构造：

```text
A = [1, dt, 0, 0;
     0, 1,  0, 0;
     0, 0,  1, dt;
     0, 0,  0, 1 - dt*b/m]

B = [0, 0;
     -r*dt/I_r, r*dt/I_r;
     0, 0;
     dt/m, dt/m]
```

其中：

```text
dt_mpc = 0.002
I_r = 1.5
r = 0.16
m = 25
b = -1000
```

### 2.2 预测方程与 QP

预测方程：

```text
X = M*x0 + C*U
```

维度：

| 符号 | 维度 |
|---|---|
| `x0` | 4 |
| `X` | (N+1)*4 = 64 |
| `U` | N*2 = 30 |
| `M` | 64×4 |
| `C` | 64×30 |
| `Q_bar` | 64×64 |
| `R_bar` | 30×30 |
| `H` | 30×30 |
| `g` | 30 |

QP 形式：

```text
min  1/2 * U' * H * U + g' * U
s.t. lb <= U <= ub
```

其中：

```text
H = C' * Q_bar * C + R_bar
g = C' * Q_bar * (M*x0 - x_ref)
```

代码里 `H/g` 没有乘 2，等价于目标函数整体缩放 1/2，最优解不变。

### 2.3 求解流程

```text
MPC_Calculate()
  -> BuildGradientVector()
  -> BuildConstraintBounds()
  -> 已初始化 ? hotstart : init
  -> hotstart 失败 ? initialize_qp
  -> getPrimalSolution(u_out)
```

只使用 `u_out(0)`、`u_out(1)`，其余预测控制量丢弃。

---

## 3. MPC 的约束现状

### 3.1 已实现

只有输入上下界：

```cpp
u_min << -150.0, -150.0;
u_max <<  150.0,  150.0;
```

在 `BuildConstraintBounds()` 中重复 N 次，形成 30 维 `lb/ub`，传给 `qp.init` / `qp.hotstart`。

### 3.2 未实现

| 约束 | 代码现状 |
|---|---|
| `20 <= F <= 150` | 没有；当前是 `-150 <= u <= 150` |
| `|ΔF| <= 30 N/step` | 没有；`last_control` 未使用 |
| `G U <= h` | 没有；`A_mat / lba / uba` 未使用 |
| 状态约束 | `x_min / x_max` 只赋值，未加入 QP |

`initialize_qp()` 和 `MPC_Calculate()` 中，`A`、`lbA`、`ubA` 传的都是 `nullptr`。

### 3.3 注意 `u_out` 不等于最终腿力

不同模式下：

```text
Follow_Gimbal_Control:
  F0 = u_out - spring + M/cos(theta) + Jump - F_mc + 5

Control_leg:
  F0 = u_out / 1.5 - spring

UP_Staris_Control_leg:
  F0 = u_out / 3.5 - spring
```

因此 `20~150 N` 是最终腿力 `F_N` 的约束，不能直接写成 `u_min = 20`；需要按补偿关系反推，或者约束最终 `F0`。

---

## 4. dt 不匹配问题

### 4.1 当前实际配置

| 项目 | 实际值 |
|---|---|
| `TIM_2ms` 实际启动周期 | `0.001 s`（1 kHz） |
| MPC 调用周期 | 1 ms |
| `dt_mpc` | 0.002 s |
| `N` | 15 |
| 名义预测时域 | `15 * 0.002 = 0.030 s` |

### 4.2 会发生什么

- QP 内部的名义预测时域是 30 ms；
- 但模型每一步按 2 ms 推进，实际控制器每 1 ms 重解一次；
- 第一个控制量只执行 1 ms，模型却以为执行了 2 ms；
- 模型会高估状态变化和控制效果，导致预测轨迹与真实系统不一致；
- 可能表现为高度/roll 响应偏硬、抖动、反复修正；
- 原来按 2 ms 调的 `Q/R/F` 在修正 `dt` 后需要重新整定；
- 1 kHz 求解比 2 ms 求解 CPU 负担翻倍。

### 4.3 其他 dt 不一致

```cpp
Chassis::dt = 0.002;
height_planner.Step(dt, ...);      // 实际 1 ms 调用一次
X_filter += Velocity_filter * (1.0f / 1000.0f);  // 这里用 1 ms
```

高度规划器和 MPC 模型都按 2 ms，但实际控制是 1 ms，存在系统性不一致。

### 4.4 推荐配置

| 目标 | 调用周期 | `dt_mpc` | `N` | 预测时域 |
|---|---|---|---|---|
| 1 kHz + 15 ms | 1 ms | 0.001 | 15 | 15 ms |
| 1 kHz + 30 ms | 1 ms | 0.001 | 30 | 30 ms |
| 2 ms + 30 ms | 2 ms | 0.002 | 15 | 30 ms |
| 100 Hz + 150 ms | 10 ms | 0.01 | 15 | 150 ms |

原则：`dt_mpc = 控制周期`，`N = 期望预测时域 / dt_mpc`。

---

## 5. 卡尔曼滤波、INS 与打滑策略

### 5.1 底盘 2 状态 KF

状态：

```text
x = [v, a]
```

参数：

| 项目 | 值 |
|---|---|
| `F` | `[1, 0.001; 0, 1]` |
| `H` | `I2` |
| `P0` | `I2` |
| `StateMinVariance` | `[1e-4, 0.03]` |

三种模式：

| 模式 | Q | R | 速度测量 |
|---|---|---|---|
| Normal | `diag(1, 2000)` | `diag(100, 1)` | `average_speed` |
| Slip | `diag(0.1, 4000)` | `diag(9000, 100)` | `kf.x_hat(0)` |
| Spin | `diag(1, 1)` | `diag(100, 8000)` | `average_speed` |

模式优先级：`spin > slip > normal`。

### 5.2 打滑策略

阈值：

```text
WHEEL_BASE = 0.25
SLIP_DVW_THRESHOLD = 0.06
SLIP_DVB_THRESHOLD = 1.20
SLIP_DBY_THRESHOLD = 0.90
SLIP_TORQUE_FACTOR = 0.60
SLIP_RECOVER_RATE = 0.05
```

流程：

1. 轮速估计 `v_wheel_l/r`；
2. 4 点滑动均值计算 `dv_wheel`；
3. 扣除车体加速度增量 `dv_acc`；
4. 外层门：yaw 差、轮速与车体速度差；
5. 内层判据 + 迟滞锁存；
6. `slip_active` 切换 KF Slip 模式；
7. `slip_factor` 乘到 LQR 的 `T[0]/T[1]`；
8. 打滑时清 `X_filter` 和 `Target_X`。

新旧版本在数值和逻辑上基本一致，只是代码位置和注释不同。

### 5.3 INS / EKF

- 旧版 `INS_Init()` 调 `QuaternionEKF_Init` 和 `Tactical_Init`；
- 旧版 `INS_Update()` 每 1 ms 跑 `Quaternion_EKF_Calculate`；
- 但旧版 EKF 输出的姿态角并没有真正用于控制，姿态一直被 mc02 的 `roll/pitch/yaw` 覆盖；
- 新版 `INS_Update(rx)` 只做数据拷贝和符号赋值：

```cpp
pitch = -rx.pitch;
yaw   =  rx.yaw;
roll  = -rx.roll;
```

- 因此控制看到的姿态等价，但新版不再运行 EKF/Tactical Fusion，节省 CPU；
- `QuaternionEKF` 源码仍在、仍参与编译，但不再被调用。

---

## 6. 检录模式与原地掉头

### 6.1 `Check_in_Control()`

用途：检录/入场模式。

- 轮子：`T_L = T_R = -Target_Velocity`，摇杆直接给轮力矩，开环；
- 髋关节：`Set_Tp(0)`，基本卸力；
- 腿力：只有超伸保护；
- `X_filter = 0`；
- 不做 MPC、LQR、roll/yaw/高度闭环。

超伸保护：

```cpp
ll = (L0/0.34)^2 - 1;  // 左腿二次
left_leg.Set_F0(-ll);

rr = L0/0.34 - 1;      // 右腿线性
right_leg.Set_F0(rr);
```

注意：

- 左右公式不一致，最大力只有约 0.25 N / 0.12 N，量级很小；
- 如果 L0 <= 0.34，力为 0；
- 这个模式不能在地面上支撑站立，适合机器人被架起/扶住时检查轮子。

### 6.2 `yaw_rotate_active`

这是原地掉头/绝对偏航旋转，不是小陀螺。

- 目标：`target_yaw_abs = current_yaw + π`，即转 180°；
- 旋转期间：平移、roll、pitch、theta 置 0，高度保持；
- 用 `PID_Fallow_yaw` 计算 `Target_Omega`；
- `|yaw_error| < 0.2 rad` 自动退出；
- 不调用 `Set_Spin_Mode`，和小陀螺不同。

当前代码里 `RC.press_ctrl` 的赋值被注释，`yaw_rotate_active` 实际不会被触发，是死代码。

---

## 7. QP 求解器与嵌入式部署

### 7.1 QP 是问题，不是求解器

当前线性 MPC 的 QP：

```text
min  1/2 U' H U + g' U
s.t. lb <= U <= ub
```

求解器：`qpOASES::QProblem`，active-set。

- `QProblem`：H/A 固定，只有 g/lb/ub 变化；
- `SQProblem`：H/A 也可变，用于 SQP/NMPC。

### 7.2 为什么 QP 在 MPC 里常见

- 二次代价 + 线性模型/约束 = 凸 QP；
- 凸问题有全局最优、可预测；
- qpOASES/OSQP/HPIPM 等实时求解器成熟；
- 热启动/代码生成适合嵌入式；
- NMPC 的 SQP 内层仍然是 QP。

### 7.3 为什么通用 QP 库不直接上 MCU

不是 QP 不能上 MCU，而是通用实现太重：

- 动态内存/堆；
- double 精度；
- C++ 标准库/iostream；
- 代码体积；
- active-set 最坏迭代次数不固定；
- 矩阵存储大。

当前 30 维 QP、`Q_bar` 64×64、`C` 64×30，双精度约几十 KB；H7 级别可以，F103 级别不行。

### 7.4 可选路线

| 方案 | 算法 | 特点 |
|---|---|---|
| qpOASES | active-set | 成熟、热启动好；需裁剪 |
| TinyMPC | ADMM | 静态内存、float、固定迭代、代码生成 |
| OSQP | ADMM | 稀疏、嵌入式友好 |
| HPIPM | 内点法 | 利用 MPC 稀疏结构 |
| FiOrdOs / ACADO / CVXGEN | 代码生成 | 专用求解器，最快最小 |
| 显式 MPC | 离线 mpQP | 在线查表，维度受限 |

TinyMPC 本身也是 QP 求解器，只是用 ADMM 和嵌入式实现。

---

## 8. 线性 MPC、SQP NMPC、CasADi NMPC 对比

| 项目 | 线性 MPC (`MPC_Control`) | SQP NMPC (`NMPC.cpp`) | CasADi NMPC (`NMPC_casADi.cpp`) |
|---|---|---|---|
| 模型 | 固定线性 A/B | 非线性，在线线性化 | 非线性 SX，RK4 |
| 优化问题 | 一个凸 QP | 一串 QP | 一个 NLP |
| 求解器 | qpOASES `QProblem` | qpOASES `SQProblem` | CasADi + IPOPT |
| 决策变量 | `U`，30 | `N(n+p)+n` | `X+U`，106（n=6,p=4,N=10） |
| 约束 | 仅 u ±150 | x/u bounds；动力学等式被注释 | 初始状态 + RK4 动力学 + x/u bounds |
| 实时性 | 好，2 ms | 中等/偏重 | 差，不适合实时 |
| 全局最优 | 是（凸） | 否 | 否 |
| 代码状态 | 在用 | 未启用、未完成 | 未启用 |

### 8.1 SQP NMPC 的关键问题

- `build_equality_constraints()` 里动力学等式全部被注释；
- `build_inequality_constraints()` 是空函数；
- 实际 QP 只有变量 bounds，以及全零的“假等式约束”；
- `build_dynamics()` 里 `theta_ddot` 除了 1000，但 `build_dynamics_ad()` 没有除，线性化与预测不一致；
- 状态注释里有 `P`，但代码没有使用 `x(10)`，需要确认 `n=10` 还是 `n=11`。

### 8.2 CasADi NMPC

- 6 状态、4 控制、N=10、dt=0.03；
- 用 `SX` 符号建模，RK4 离散；
- IPOPT：`max_iter=100`、`tol=1e-3`、`acceptable_tol=1e-2`、`expand=true`；
- 终端只用 `Q`，没有单独 `F`；
- `solve()` 传空 guess，没有跨周期热启动；
- 适合离线验证，不适合直接上 MCU。

---

## 9. LQR 与 MPC 的关系

### 9.1 当前代码的分工

LQR：

- 输入：`err[0..9]`，包括水平位置/速度、yaw 角速度、左右腿角度/角速度、pitch/pitch rate；
- 输出：`T[0..3]`，左右轮力矩 + 左右髋力矩；
- 作用：平衡、速度、yaw、腿角。

MPC：

- 输入：`[roll, roll_rate, height, height_rate]`；
- 输出：`u_out(0/1)`，左右腿虚拟腿力；
- 作用：高度、roll、约束、预测。

在 `Follow_Gimbal_Control()` 中：

```text
轮矩 = LQR T[0]/T[1]
髋矩 = LQR T[2]/T[3]
腿力 = MPC u_out(0/1)
```

两者最后通过 `VMC_Calc_2()` 合成关节力矩，但不是同一个控制通道。

### 9.2 是否重复

不重复：

- 两者使用不同降阶模型/不同目标；
- LQR 离线算固定增益 K；
- MPC 在线解 QP；
- 当前 MPC 模型没有轮子/yaw/水平速度，不能替代 LQR。

如果做全状态 MPC，包含轮矩/髋矩/腿力，则 MPC 可以替代 LQR 作为主控；但 LQR 仍可用于保底、终端代价、热启动、内环。

---

## 10. 终端代价、终端集、热启动

### 10.1 终端代价

MPC 代价：

```text
J = sum_{k=0}^{N-1} (x_k'Qx_k + u_k'Ru_k) + x_N' F x_N
```

如果 `F = P`，其中 `P` 是 LQR 的 Riccati 解，则终端代价近似“从 x_N 开始的无限时域最优代价”。

### 10.2 终端控制器与终端集

- 终端控制器：在 x_N 之后用 `u = Kx`；
- 终端集 Ω：在 `u=Kx` 下正不变，且满足 x/u 约束；
- 终端约束：`x_N ∈ Ω`；
- 作用：保证 N 步之后 LQR 能安全接管，从而证明递归可行性。

### 10.3 热启动

LQR rollout：

```text
x = x0
for k = 0..N-1:
    u[k] = clamp(-K*x, u_min, u_max)
    x = A*x + B*u[k]
```

得到整条未来 N 步轨迹，作为 MPC/NMPC 的初始猜测。

- 给的是整个 horizon 的初值，不是只给第 0 步；
- 优化器对所有步同时迭代；
- 优化后仍然只执行 `u0*`；
- 热启动减少迭代次数，不改变问题规模。

---

## 11. MATLAB LQR 脚本分析

文件：

`/home/zhc/App_data/QQ_data/HerKules_VOCAL_SJ_LQR_v4_with_data.m`

### 11.1 脚本流程

1. 符号建模 10 状态、4 控制的连续 A/B；
2. 输入物理参数、腿部数据集、`lqr_Q(10×10)`、`lqr_R(4×4)`；
3. 对每对腿长 `(l_l, l_r)` 调用 `get_K_from_LQR`；
4. 用 `icare` 解连续 Riccati，得到 `P/K`；
5. 采样 40 个增益，做 `poly22` 拟合；
6. 输出 `K_Fit_Coefficients`，大小 40×6。

### 11.2 哪些进入 C 代码

| MATLAB 部分 | 是否进入 C |
|---|---|
| 符号 A/B | 否 |
| 物理参数、Q/R | 否 |
| `K_sample` | 否 |
| `P`（Riccati 解） | **否，被丢弃** |
| `L_k` | 否 |
| `K_Fit_Coefficients` | **是，对应 C 的 `P[40][6]`** |
| Step 2.5 打印的 K | 仅调试 |

C 代码：

```text
P[40][6]
  -> Fitting_K_Calc()
  -> Fitting_K[4][10]
  -> LQR_Calc()
  -> T[0..3]
```

### 11.3 Riccati 的 P 用到了吗

没有。

```matlab
[P,K,L_k] = icare(A_ac,B_ac,lqr_Q,lqr_R,[],[],[]);
```

- `P` 被算出；
- 函数只 `return K`；
- `P` 没有进入拟合；
- C 代码没有 `P`。

### 11.4 `P[40][6]` 是什么

不是 Riccati P，而是 LQR 增益的拟合系数：

```text
4 个控制 × 10 个状态 = 40 个增益
每个增益拟合为 l_l、l_r 的二次函数
每个二次函数 6 个系数
-> 40×6
```

拟合公式：

```text
K_ij(l_l,l_r) = p00 + p10*l_l + p01*l_r
              + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
```

### 11.5 如果要用 P 做 MPC 终端代价

不能直接用这个 10 状态连续 LQR 的 P，因为：

- 维度不同：10×10 vs MPC 的 4×4；
- 连续 vs 离散：`icare` 是连续，MPC 是离散；
- 模型不同：MATLAB 是 10 状态全状态模型，MPC 是 4 状态高度/roll 模型。

正确做法：

```matlab
% 用 MPC 自己的 A/B/Q/R
[K_mpc, P_mpc] = dlqr(A_mpc, B_mpc, Q_mpc, R_mpc);
F = P_mpc;
```

---

## 12. 关键参数汇总

### 12.1 线性 MPC

```text
n = 4
p = 2
N = 15
dt_mpc = 0.002
Q = diag(1.5e7, 1e3, 1.5e7, 1e3)
F = diag(1.5e8, 1e4, 1.5e8, 1e4)
R = 0.9 * I2
u_min = [-150, -150]
u_max = [ 150,  150]
x_min = [-pi/4, -0.5, 0.1, -1.0]
x_max = [ pi/4,  0.5, 0.38, 1.0]  (未加入 QP)
```

### 12.2 底盘 KF

```text
F = [1, 0.001; 0, 1]
H = I2
P0 = I2
StateMinVariance = [1e-4, 0.03]

Normal: Q=diag(1,2000),   R=diag(100,1)
Slip:   Q=diag(0.1,4000), R=diag(9000,100)
Spin:   Q=diag(1,1),      R=diag(100,8000)
```

### 12.3 打滑阈值

```text
WHEEL_BASE = 0.25
SLIP_DVW_THRESHOLD = 0.06
SLIP_DVB_THRESHOLD = 1.20
SLIP_DBY_THRESHOLD = 0.90
SLIP_TORQUE_FACTOR = 0.60
SLIP_RECOVER_RATE = 0.05
```

---

## 13. 风险与待办

1. **dt 不匹配**：1 kHz 实际周期 vs `dt_mpc=0.002`，需要统一。
2. **MPC 约束不足**：`20~150 N`、`|ΔF| <= 30`、`G U <= h` 尚未实现。
3. **`u_out` 与最终 F0 的关系**：不同模式有 `/1.5`、`/3.5`、弹簧/重力补偿，约束应加在最终量上。
4. **`Control_leg()` 的 `x_ref` 最后一块为 0**：终端代价 `F` 会把 `x_N` 拉向 0，需检查是否符合预期。
5. **`x_min/x_max` 未加入 QP**：状态约束目前不生效。
6. **`last_control` 未使用**：无法做力变化率约束/简单热启动。
7. **NMPC.cpp 未完成**：动力学等式被注释，`/1000` 在 AD 版本缺失，状态 `P` 未定义。
8. **CasADi NMPC 未启用**：6 状态模型，不是 10 状态全状态模型；无跨周期热启动。
9. **QP 求解失败处理**：`getPrimalSolution` 前未检查 `solver_status`，失败时可能用旧值。
10. **timer 命名**：`timer_2ms.start(0.001)` 实际是 1 ms，容易误判。
11. **yaw_rotate 死代码**：`RC.press_ctrl` 被注释，模式不会触发。
12. **检录模式超伸力很小**：左右公式不一致，最大约 0.25 N / 0.12 N。

---

## 14. 关键文件索引

| 文件 | 作用 |
|---|---|
| `src/Algorithm/MPC/MPC_Control.h` | 线性 MPC 类定义（已加中文注释） |
| `src/Algorithm/MPC/MPC_Control.cpp` | 线性 MPC 实现（已加中文注释） |
| `src/Algorithm/MPC/NMPC.h/.cpp` | SQP NMPC 原型（未启用） |
| `src/Algorithm/MPC/NMPC_casADi.h/.cpp` | CasADi + IPOPT NMPC（未启用） |
| `src/Algorithm/KalmanFilter/KalmanFilter.h/.cpp` | 2 状态底盘 KF |
| `src/Algorithm/EKF/QuaternionEKF.*` | 四元数 EKF（源码保留，运行时不调用） |
| `src/Devices/INS.cpp` | INS 数据拷贝/姿态赋值 |
| `src/Interaction/Chassis.cpp` | MPC/LQR 调用、KF、打滑、检录、掉头 |
| `src/Task/task_and_callback.cpp` | 1 ms/2 ms/10 ms 周期任务 |
| `src/Interaction/robot.cpp` | 模式选择、遥控输入、目标值 |
| `src/Algorithm/VMC/VMC&LQR_Calc.cpp` | VMC/LQR 计算、`Fitting_K_Calc` |
| `src/Algorithm/VMC/lqr_k_calc.c` | LQR 增益系数 |

---

## 15. 10 状态全状态线性 MPC（2026-10-27 落地）

> 本节记录把 SQP NMPC 换成「10 状态线性 MPC + QP」的完整方案与已落地代码。
> 对应文件：`src/Algorithm/MPC/Robot_MPC.h`、`Robot_MPC.cpp`、`Chassis::MPC_Robot_Calc()`、`TIM_4ms_PeriodElapsedCallback()`。

### 15.1 为什么放弃 SQP NMPC

| 项 | SQP NMPC（`NMPC.cpp`） | 10 状态线性 MPC（`Robot_MPC`） |
|---|---|---|
| 决策变量 | `N(n+p)+n` = **150**（N=10） | `4N` = **60**（N=15） |
| 等式约束 | **110**（每步动力学） | **0**（condensing 消掉） |
| 每周期计算 | CppAD 重建雅可比 + 10 次 SQP 迭代 + 线搜索 | 1 次 `g` + 1 次 QP |
| 实测（开发机） | 4 ms 任务跑不动 | 热路径 **~53 μs**，重建 **~350 μs** |

### 15.2 模型来源：不是“拟合 A、B”

**结论：A、B 是腿长 L 的有理函数，用 5×5 精确重建，不需要拟合。**

5 个牛顿方程（MATLAB 脚本 eqn1–eqn5）可写成：

$$M(L)\,\ddot q = E\,u + G\,\theta,\qquad q=[\theta_{wl},\theta_{wr},\theta_{ll},\theta_{lr},\theta_b]$$

在平衡点（$\theta=0$、速度=0、$u=0$）处 $\ddot q=0$，故

$$\frac{\partial\ddot q}{\partial\theta}=M^{-1}G,\qquad \frac{\partial\ddot q}{\partial u}=M^{-1}E$$

这两个矩阵**就是** MATLAB 里 `jacobian` 得到的 `J_A`、`J_B`。所以 C 里只要建 5×5 矩阵解一次，就得到与 MATLAB `subs` 完全一致的 A、B，对任意腿长都成立。

真正的“拟合/插值”只发生在两处：

1. **腿长 → 3 个物理参数**：`L → (l_w, l_b, I_l)`，用 MATLAB 的 `Leg_data_l` 表线性插值（24 个点，1 cm 间隔）；
2. **（可选）控制器调度**：K、P 随腿长变化，可以像原来那样 poly22 拟合，也可以在 C 里迭代 DARE（当前实现，见 15.5）。

**不要再对 A、B 的元素做 poly22 曲面拟合**：它们是 $1/L$、$1/L^2$ 型的量，多项式拟合误差大，而且会破坏 A 的零结构。

### 15.3 状态用「误差态」，与 LQR 完全同构

`Chassis::feedback_update()` 里已经算好了 `err[10]`：

| 下标 | 含义 | 现 LQR 的写法 |
|---|---|---|
| 0 | 水平位移误差 | `X_filter - Target_X` |
| 1 | 水平速度误差 | `Velocity_filter - Target_Velocity` |
| 2 | 偏航角误差 | 恒 0（当前禁用） |
| 3 | 偏航角速度误差 | `INS->gyro[2] - Target_Omega` |
| 4/5 | 左腿角/角速度误差 | `left_leg.Get_theta() - Target_Theta` |
| 6/7 | 右腿角/角速度误差 | `right_leg.Get_theta() - Target_Theta` |
| 8/9 | 机体俯仰角/角速度误差 | `INS->pitch`、`INS->gyro[0]` |

这个顺序**正好就是** MPC 的状态顺序。所以 MPC 直接对误差态做优化：

$$e_{k+1}=A_d e_k+B_d u_k,\qquad J=\sum_{k=0}^{N-1}\left(e_k^TQe_k+u_k^TRu_k\right)+e_N^TFe_N$$

好处：不需要另建参考轨迹；而且由 15.5 的 $F=P$，**无约束时 $u_0=-Ke$，与 LQR 逐位相同**，切换天然无扰。

### 15.4 代码结构与数据流

```text
每周期 Run()（4 ms）                     Rebuild()（腿长变 >2mm 或 100ms）
┌────────────────────────┐               ┌──────────────────────────┐
│ ① 存 e0，腿长限幅        │               │ ① 5×5 → Ac, Bc           │
│ ② 变化>2mm? ───────────┼──→ Rebuild ──→│ ② 二阶 Taylor → Ad, Bd   │
│ ③ g = CtQ·Mp·e0        │               │ ③ DARE → F(=P), K        │
│ ④ 更新 lb/ub/lbA/ubA    │               │ ④ condensing → Mp, Cp    │
│ ⑤ init/hotstart        │               │ ⑤ CtQ, H                 │
│ ⑥ 成功 → u=U(0..3)      │               │ ⑥ Δu 约束矩阵 Am         │
│   失败 → Fallback       │               │ ⑦ 只准备数据，不 init     │
└────────────────────────┘               └──────────────────────────┘
```

### 15.5 六个关键设计决策

**① $F=P$（离散 Riccati 解）**
`Dare()` 迭代 `P ← Q + AclᵀPAcl + KᵀRK` 直到收敛。$P$ 用上一周期的值热启动，通常 3~10 次迭代。
它同时给出兜底增益 $K$（与 LQR 同源）。
**注意尺度**：$P$ 必须与 QP 里的 Q、R 同源（同一次 DARE）。把 `icare` 的连续 P 直接塞进离散 QP，终端代价会差 $1/dt$ 倍（4 ms 时约 250 倍），等于没有终端代价。

**② condensing**
把 16 组未来状态用 `e0` 和 `U` 表示：`X = Mp·e0 + Cp·U`。变量从 150 降到 60，且**没有等式约束**——这是 active-set 能快一个数量级的主因。

**③ 约束：只有 u 盒约束 + Δu**
- `|T| ≤ [15, 15, 60, 60] Nm`（★按实测峰值修改★）
- `|u_k − u_{k−1}| ≤ [1, 1, 2, 2] Nm/步`
只有盒约束时可行域永远非空，**QP 不会 infeasible**；加状态约束（$|\theta_b|$）时才需要软化。

**④ SQProblem + hotstart**
因为 $A_d,B_d$ 随腿长变 → $H$ 变，所以用 `SQProblem`（`QProblem` 要求 H 固定）。
`Rebuild()` 只准备数据，**不调用 `init`**；`Run()` 里首次 `init`、之后一律 `hotstart`（支持 H 变化），失败才冷启动重来。
早期版本每次重建都 `init`，冷启动要 60 次迭代，约 2 ms；改完后重建降到 ~350 μs。

**⑤ CtQ 分块计算**
`CtQ = Cpᵀ·Qb`，而 `Qb` 是块对角的（前 15 块是**对角阵** Q，最后一块是满阵 F）。按块算把 150 万次乘法降到不到 2 万次，这是重建提速的第二大来源。

**⑥ 锁下沉到 `MPC_Robot_Calc()` 内部**
TIM_4ms 与 TIM_1ms/2ms 是两个线程，共用 `mc_02->rx_mutex`。`Rebuild()` 最长约 0.35 ms，如果像其他任务那样在回调最外层加锁，1 kHz 任务会被堵住。所以锁只包住两次数据拷贝（各 < 1 μs），求解全程不持锁。

### 15.6 参数

| 项 | 值 | 说明 |
|---|---|---|
| `n / p / NP` | 10 / 4 / 15 | NP × dt = 60 ms 预测时域 |
| `dt` | 0.004 s | 必须等于 TIM_4ms 周期 |
| `Q` | diag(6000, 90, 30000, 1500, 19000, 300, 19000, 300, 600000, 500) | 沿用 LQR |
| `R` | diag(200, 200, 8, 8) | 沿用 LQR |
| `F` | DARE 解 | 由 `Dare()` 在线算 |
| `u_min/u_max` | ∓[15,15,60,60] | 占位，**必须改成真实限幅** |
| `du` | [1,1,2,2] Nm/步 | 4 ms 下相当于 250/500 Nm/s |
| 重建阈值 | 2 mm / 100 ms | `Run()` 里判断 |
| `nWSR` | 600（init）/ 600（hotstart） | 迭代上限 = 最坏耗时上界 |

### 15.7 自测数据（开发机，`-O2`）

```text
max|eig(Ad)|            = 1.0980        （腿摆模态 ~23.4 rad/s，dt·λ≈0.094）
B(1,0)+B(1,1)           = 10.98         （两轮同向 1 Nm 时的 s̈）
eig(F) min/max          = 754.7 / 1.39e7
Run() 热路径            ≈ 53 μs         （2000 次平均）
Run() 触发一次重建       ≈ 350 μs
无约束一致性             u = [0.605, 0.605, 0.404, 0.404]
                        Fallback = [0.597, 0.597, 0.448, 0.448]（Δu 约束导致的小差异）
大偏差（θ_b=0.6）         u = [1, 1, 2, 2]（正好贴 Δu 限幅，符合预期）
```

### 15.8 上机前必做

**① 用 MATLAB 对模型（最重要）**
把 `HerKules_VOCAL_SJ_LQR_v4_with_data.m` 的 `l_l_ac/l_r_ac` 固定成 0.1819 与对应的
`l_wl=0.13556, l_bl=0.09331, I_ll=0.04695652`，运行后打印 `double(subs(A,...))`、`double(subs(B,...))`，
与下表逐元素对比（表中是精确解，C 侧在 L=0.1819 时应一致到 1e-4）：

```text
A（L=0.1819，只列非零项）
 A(1,2)=1
 A(2,5)=A(2,7)=-47.3927
 A(3,4)=1
 A(4,5)=-0.0071, A(4,7)=+0.0071
 A(5,6)=1
 A(6,5)=535.8784, A(6,7)=9.7053
 A(7,8)=1
 A(8,5)=9.7053,  A(8,7)=535.8784
 A(9,10)=1
 A(10,5)=A(10,7)=-3.0523, A(10,9)=13.4382

B（列 = [T_wl, T_wr, T_bl, T_br]）
 B(2,:)  = 5.5158,  5.5158,  -1.3466, -1.3466
 B(4,:)  = -5.9386, 5.9386,  -0.0002,  0.0002
 B(6,:)  = -51.0182, -7.8335, 15.2267, 0.2758
 B(8,:)  = -7.8335, -51.0182, 0.2758, 15.2267
 B(10,:) = -0.2236, -0.2236, -1.1284, -1.1284
```

**② 无约束一致性**
把 `u_min/u_max` 临时设成 ±1e6、`du` 设成 1e6，此时必须满足 `u0 ≈ -K·e`（K 与 Fallback 一致）。对不上就是索引/权重有错。

**③ 只算不发**
先把 `mpc_enable` 设成 `false`，或只打印 `T_mpc`，确认单周期耗时与抖动。

**④ 台架接管**
机器人架起，`mpc_enable = true`，观察腿力矩方向是否与原 LQR 一致（重点是髋矩 $T_{bl}/T_{br}$ 的符号）。

### 15.9 已知风险与待办

1. **限幅是占位值**：±15 Nm（轮）/±60 Nm（髋）必须换成电机+减速箱的真实输出限幅。
2. **增益手感与原 LQR 不同**：现 LQR 在 `LQR_Calc()` 里对通道乘了 0.9/0.95/3.0/1.4 的经验系数，而 MPC 的 K 直接来自 Q/R。若发现俯仰偏软，优先调 `Q(8,8)`。
3. **1 kHz 回调里的 `std::cout`**：`TIM_2ms_PeriodElapsedCallback()` 里每毫秒打印一次耗时，会严重破坏实时性，**上机前必须删掉或改成环形缓冲**。
4. **符号约定**：模型假设两条腿同号。若髋矩方向不对，先查 `right_leg` 的 VMC 符号。
5. **`use of age > 0.1` 强制重建**：即使腿长不变也每 100 ms 重算一次 DARE，属于保守设置；若 CPU 紧张可以放宽到 0.5 s。
6. **尚未接入的约束**：$|\theta_b|$ 状态约束、松弛变量、腿力下限——按 §3.3/§3.4 逐步加。
7. **本 MPC 只替代 LQR**：10 状态模型里没有腿长/腿力，高度仍由 4 状态 `MPC_body` 负责，两者通过 `VMC_Calc_2` 合成。

### 15.10 本次改动清单

| 文件 | 改动 |
|---|---|
| `src/Algorithm/MPC/Robot_MPC.h` | 新增：10 状态 MPC 类定义 |
| `src/Algorithm/MPC/Robot_MPC.cpp` | 新增：5×5 建模、DARE、condensing、QP 求解 |
| `src/Interaction/Chassis.h` | `#include "Robot_MPC.h"`；`mpc_robot`、`mpc_enable`、`T_mpc[4]`、`mpc_ok`、`MPC_Robot_Calc()` |
| `src/Interaction/Chassis.cpp` | `Init()` 里 `mpc_robot->Init(0.004)`；新增 `MPC_Robot_Calc()`；`Follow_Gimbal_Control()` 里用 `T_mpc` 覆盖 `T[0..3]`（轮矩仍乘打滑系数） |
| `src/Task/task_and_callback.cpp` | `timer_10ms` → `timer_4ms`，周期 0.01 → **0.004**；回调改为调用 `MPC_Robot_Calc()` |

> 注意：`CMakeLists.txt` 用 `file(GLOB_RECURSE ...)`，新增源文件后需要重新跑一次 `cmake -S . -B build`，否则 `Robot_MPC.cpp` 不会进编译。

### 15.11 一句话

> 10 状态 MPC = 5×5 精确建模 + condensing 成 60 变量 QP + $F=P$ 终端代价；腿长不变时每周期只算一次 $g$ 和一次热启动 QP（~50 μs），腿长变化时重建一次（~350 μs）。它替代的是 LQR，高度仍归 4 状态 MPC。

---

## 16. 复核与修复（2026-10-27）

> 本节是对 §15 落地代码的复核结果。**§16.5 的耗时与增益数据取代 §15.6/§15.7。**

### 16.1 机体参数比对：全部一致

| 参数 | C（`Robot_MPC.cpp`） | MATLAB（`Step 2`） | 结论 |
|---|---|---|---|
| `R_w` | 0.06 | 0.06 | ✅ |
| `R_l` | 0.33 | 0.33 | ✅ |
| `l_c` | 0.037 | 0.037 | ✅ |
| `m_w` | 0.26 | 0.26 | ✅ |
| `m_l` | 2.6183599 | 2.6183599 | ✅ |
| `m_b` | 35.542 | 35.542 | ✅ |
| `I_w` | 7.51e-7 | 7.51e-7 | ✅ |
| `I_b` | 0.960 | 0.960 | ✅ |
| `I_z` | 0.926 | 0.926 | ✅ |
| `g` | 9.81 | 9.81 | ✅ |

### 16.2 腿长表比对：24 行完全一致

`LEG[24][4]` 与 `Leg_data_l` 逐行逐列比较，最大差值 **0**。

### 16.3 问题 1：DARE 迭代不足，K 是错的

Riccati 迭代在**积分方向**（位置/速度）收敛极慢：闭环极点约 0.996，误差按 $0.996^{2k}$ 衰减，收敛到 1e-9 需要约 **2600 次**。

原来只给 80 次，结果：

| 通道 | 你的 `P[40][6]` 反算 | 80 次迭代 | 判定 |
|---|---|---|---|
| 轮矩/俯仰 | −31.53 | −29.14 | 接近（该模态收敛快） |
| 髋矩/俯仰 | −128.56 | −126.96 | 接近 |
| 轮矩/位移 | −2.678 | **+0.031** | **差 86 倍，符号也错** |
| 轮矩/速度 | −3.112 | **−0.020** | **差 155 倍** |

交叉验证：$b\approx5.5$、$q_1=6000$、$r=200$ 的双积分器，位置增益解析值约 5.47，与你的表（2.68）同量级，与我当时的 0.031 差两个数量级——**是迭代不够，不是表错**。

**修复**：`Dare()` 增加 `maxit` 参数。

- **冷启动**（`Init` 时一次）：3000 次，约 **5.5 ms**（只在上电发生一次）
- **重建**（腿长变 2 mm）：40 次热启动**跟踪**，因为 P 随 L 变化很小

跟踪精度验证：连续 5 次 +2 mm 重建后，与同腿长冷启动相比 **K 最大相对差 0.12%**。

### 16.4 问题 2：力矩方向与现 LQR 相反（更危险）

`LQR_Calc()` 用的是 `T = Fitting_K · e`，而 MATLAB `icare` 的 K 对应 `u = -Kx`。两者自洽只有一种解释：**电机力矩正方向与模型里 $T$ 的正方向相反**。

用俯仰通道验证：模型 $A(9,8)=+13.44$、$B(9,\cdot)=-1.13$。

- 若按模型约定输出 `u = -K_dare·e`：$e_8>0 \Rightarrow u>0 \Rightarrow$ 俯仰角加速度为负 → 正确
- 现场 `T = K_fit·e`，$e_8>0 \Rightarrow T<0 \Rightarrow$ 模型里俯仰角加速度为正 → 会发散

它能稳定，说明硬件方向是反的。所以 MPC 若直接输出 `-K_dare·e`，与现 LQR **符号相反**，上机必发散。

**修复**：`BuildAB()` 末尾把 `Bc` 取负，让 QP 的解 `u` 直接就是「发给电机的力矩」。

```cpp
constexpr bool kHwTorqueFlipped = true;
if (kHwTorqueFlipped) Bc = -Bc;
```

若上机发现方向不对，把 `kHwTorqueFlipped` 改成 `false` 即可。

### 16.5 修复后的对照数据（L = 0.18）

| 通道 | `K_fit`（现场用） | 修复前实际施加 | **修复后实际施加** |
|---|---|---|---|
| 位移→轮矩 | −2.678 | −0.031 | **−2.101** |
| 速度→轮矩 | −3.112 | +0.020 | **−2.456** |
| 偏航→轮矩 | −7.616 | +6.124 | **−7.248** |
| 左腿角→轮矩 | −12.562 | +9.731 | **−10.727** |
| 俯仰→轮矩 | −31.531 | +29.140 | **−30.221** |
| 左腿角→髋矩 | +75.515 | −57.336 | **+62.577** |
| 俯仰→髋矩 | −128.555 | +126.960 | **−122.144** |

**符号全部对齐，量级差 4%~30%**（poly22 拟合误差 + 连续/离散之差）。

耗时（开发机 `-O2`）：

```text
Init（冷启动 DARE）      5.5 ms      只在上电一次
Run 热路径              49.6 μs
重建 +2 mm              440~590 μs
同腿长（100 ms 到期）    50~190 μs
温度跳到 +7 cm           ~560 μs
```

### 16.6 仍然待确认的三件事

1. **限幅是占位值**：现在 ±5 Nm（轮）/ ±40 Nm（髋）。`LQR_Calc()` 里对速度项单独限到 ±16 Nm，说明轮矩正常工作值可能超过 5 Nm。建议先打点统计 `T[0..3]` 的 99% 分位，再把 `u_min/u_max` 设成略高于它。
2. **Δu 起步可能过严**：`du = 1 Nm/步`（4 ms）意味着 250 Nm/s，0→10 Nm 需要 40 ms，急停/侧推时可能不够。建议先用 5 Nm/步（1250 Nm/s）跑通，再逐步收紧。
3. **1 kHz 回调里的 `std::cout`**：`TIM_2ms_PeriodElapsedCallback()` 每毫秒打印一次，会毁掉 4 ms 抖动，上机前必须删。

### 16.7 上机验证步骤（按顺序）

1. **台架悬空**，`mpc_enable = true`，用手扶住机器人，只打点不接管：打印 `T_mpc[0..3]` 与 `T[0..3]`。
2. **符号对比**：人为给一个小俯仰（约 2°），看 `T_mpc` 与 LQR 的 `T` **同号**。若反了，把 `kHwTorqueFlipped` 取反。
3. **幅值对比**：同号后看量级，`T_mpc` 应在 `T` 的 0.7~1.3 倍之间。偏差大就先查 `Q/R`，不要急着改结构。
4. **无约束自检**：`u_min/u_max` 临时设 ±1e6、`du` 设 1e6，`T_mpc` 应等于 `-K·e`。
5. **地面接管**：先低速、低高度，确认不发生低速抖动后再放开。

### 16.8 一句话

> 参数和腿长表完全一致；但原实现有两个会直接导致上机发散的 bug：**DARE 只迭代 80 次（积分方向远未收敛）**、**力矩符号与现 LQR 相反**。两处已修，修复后的等效增益与现场 `K_fit` 符号全对、量级差 4%~30%。

---

## 附：一句话总结

> 当前 `leg_hero_27` 是“1 kHz 控制、4 状态/2 控制/15 步线性 MPC + 2 状态底盘 KF + LQR 轮矩/髋矩”的架构。MPC 目前只有 ±150 输入饱和，尚未接入腿力下限、力变化率和 `G U <= h` 约束；`dt_mpc=0.002` 与实际 1 ms 周期不匹配；LQR 与 MPC 分工不同，不是重复控制；Riccati 的 `P` 在当前 MATLAB 脚本和 C 代码中都没有被使用，C 使用的只是 LQR 增益 `K` 的拟合系数。
