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

## 附：一句话总结

> 当前 `leg_hero_27` 是“1 kHz 控制、4 状态/2 控制/15 步线性 MPC + 2 状态底盘 KF + LQR 轮矩/髋矩”的架构。MPC 目前只有 ±150 输入饱和，尚未接入腿力下限、力变化率和 `G U <= h` 约束；`dt_mpc=0.002` 与实际 1 ms 周期不匹配；LQR 与 MPC 分工不同，不是重复控制；Riccati 的 `P` 在当前 MATLAB 脚本和 C 代码中都没有被使用，C 使用的只是 LQR 增益 `K` 的拟合系数。
