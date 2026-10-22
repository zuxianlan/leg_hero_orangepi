// /**
//   ******************************************************************************
//   * @file           : NMPC_casADi.cpp
//   * @author         : Chen Haoran
//   * @brief          : None
//   * @attention      : None
//   * @date           : 2025/12/1
//   ******************************************************************************
//   */
//
// /* Includes ------------------------------------------------------------------*/
// #include "NMPC_casADi.h"
//
// /* Define --------------------------------------------------------------------*/
//
// /* Enum ----------------------------------------------------------------------*/
//
// /* Variable && Struct --------------------------------------------------------*/
//
// /* Function Declaration ------------------------------------------------------*/
//
// /* Function ------------------------------------------------------------------*/
//
// using namespace casadi;
//
// class_NMPC_casADi::class_NMPC_casADi()
// {
//     // initialize variables
//     n = 10;
//     p = 4;
//     N = 10;
//     dt = 0.1;
//
//     x_min = {-0.26, -0.05, -INFINITY, -0.3, -0.26, -0.05, -INFINITY, -0.3, -0.087, -0.5};
//     x_max = {0.26,  0.05,  INFINITY,  0.3,  0.26,  0.05,  INFINITY,  0.3,  0.087,  0.5};
//     u_min = {-1, -1, -1, -1};
//     u_max = { 1,  1,  1,  1};
//
//     build_dynamics();
//     build_optimizer();
// }
//
// void class_NMPC_casADi::build_dynamics()
// {
//     // // state variables
//     // SX x = SX::sym("x", n);
//     //
//     // SX L_theta = x(0);
//     // SX L_theta_dot = x(1);
//     // SX L_x = x(2);
//     // SX L_x_dot = x(3);
//     // SX R_theta = x(4);
//     // SX R_theta_dot = x(5);
//     // SX R_x = x(6);
//     // SX R_x_dot = x(7);
//     // SX phi = x(8);
//     // SX phi_dot = x(9);
//     //
//     // // control variables
//     // SX u = SX::sym("u", p);
//     //
//     // SX L_T = u(0);
//     // SX L_Tp = u(1);
//     // SX R_T = u(2);
//     // SX R_Tp = u(3);
//     //
//     // // other variables
//     // SX L0 = SX::sym("L0");
//     // SX L1 = SX::sym("L1");
//     //
//     // // ============ calculate L_x_ddot ============ //
//     // SX xdd_num = (5.82809 * L0 * L0 + 0.000206091) * L0
//     //     * L_theta_dot * L_theta_dot - sin(L_theta)
//     //     - 28.9519 * L0 * L0 * sin(2 * L_theta)
//     //     + 343.597 * L0 * L0 * L_T
//     //     + L0 * (11.4952 * L_T - 11.4952 * L_Tp) * cos(L_theta)
//     //     + 0.0121501 * L_T;
//     //
//     // SX xdd_den = 6.90856 * L0 * L0 * sin(L_theta) * sin(L_theta)
//     //     + L0 * L0 * cos(L_theta) * cos(L_theta)
//     //     + 0.000244298;
//     //
//     // SX L_x_ddot = xdd_num / xdd_den;
//     //
//     // // ============ calculate L_theta_ddot ============ //
//     // SX sec_theta = 1.0 / cos(L_theta);
//     // SX tan_theta = sin(L_theta) / cos(L_theta);
//     //
//     // SX thdd_num = sec_theta * (9.93531 * L0 * tan_theta
//     //         - 50.4216 * L0 * L_T
//     //         + (1.97239 * L_Tp - 1.97239 * L_T) * sec_theta)
//     //     - 0.855252 * pow(L0, 2) * pow(L_theta_dot, 2) * tan_theta;
//     //
//     // SX thdd_den = pow(L0, 2) * (1.0 * pow(tan_theta, 2) + 0.144748)
//     //     + 0.0000353616 * pow(sec_theta, 2);
//     //
//     // SX L_theta_ddot = thdd_num / thdd_den;
//     //
//     // // ============ calculate R_x_ddot ============ //
//     // SX R_xdd_num = (5.82809 * L1 * L1 + 0.000206091) * L1
//     //     * R_theta_dot * R_theta_dot - sin(R_theta)
//     //     - 28.9519 * L1 * L1 * sin(2 * R_theta)
//     //     + 343.597 * L1 * L1 * L_T
//     //     + L1 * (11.4952 * R_T - 11.4952 * R_Tp) * cos(R_theta)
//     //     + 0.0121501 * R_T;
//     //
//     // SX R_xdd_den = 6.90856 * L1 * L1 * sin(R_theta) * sin(R_theta)
//     //     + L1 * L1 * cos(R_theta) * cos(R_theta)
//     //     + 0.000244298;
//     //
//     // SX R_x_ddot = R_xdd_num / R_xdd_den;
//     //
//     // // ============ calculate R_theta_ddot ============ //
//     // SX R_sec_theta = 1.0 / cos(R_theta);
//     // SX R_tan_theta = sin(R_theta) / cos(R_theta);
//     //
//     // SX R_thdd_num = R_sec_theta * (9.93531 * L1 * R_tan_theta
//     //         - 50.4216 * L1 * R_T
//     //         + (1.97239 * R_Tp - 1.97239 * R_T) * R_sec_theta)
//     //     - 0.855252 * pow(L1, 2) * pow(R_theta_dot, 2) * R_tan_theta;
//     //
//     // SX R_thdd_den = pow(L1, 2) * (1.0 * pow(R_tan_theta, 2) + 0.144748)
//     //     + 0.0000353616 * pow(R_sec_theta, 2);
//     //
//     // SX R_theta_ddot = R_thdd_num / R_thdd_den;
//     //
//     // // ============ calculate phi_ddot ============
//     // SX phi_ddot = 2953.84 * (L_Tp + R_Tp) / 2;
//     //
//     // SX part1 = vertcat(
//     //     L_theta_dot,
//     //     L_theta_ddot,
//     //     L_x_dot,
//     //     L_x_ddot,
//     //     R_theta_dot,
//     //     R_theta_ddot
//     // );
//     // SX part2 = vertcat(
//     //     R_x_dot,
//     //     R_x_ddot,
//     //     phi_dot,
//     //     phi_ddot
//     // );
//     // SX x_dot = vertcat(
//     //     part1,
//     //     part2
//     // );
//     //
//     // dynamics = Function("dynamics", {x, u, L0, L1}, {x_dot});
//
//
//      // 状态变量
//     SX x = SX::sym("x", n);
//     SX u = SX::sym("u", p);
//     SX L0 = SX::sym("L0");
//     SX L1 = SX::sym("L1");
//
//     // 提取状态
//     SX L_theta = x(0);
//     SX L_theta_dot = x(1);
//     SX L_x = x(2);
//     SX L_x_dot = x(3);
//     SX R_theta = x(4);
//     SX R_theta_dot = x(5);
//     SX R_x = x(6);
//     SX R_x_dot = x(7);
//     SX phi = x(8);
//     SX phi_dot = x(9);
//
//     SX L_T = u(0);
//     SX L_Tp = u(1);
//     SX R_T = u(2);
//     SX R_Tp = u(3);
//
//     // 预计算常用表达式
//     SX L0_sq = L0 * L0;
//     SX L1_sq = L1 * L1;
//     SX sin_L_theta = sin(L_theta);
//     SX cos_L_theta = cos(L_theta);
//     SX sin_R_theta = sin(R_theta);
//     SX cos_R_theta = cos(R_theta);
//
//     // 计算左腿动力学（简化表达式）
//     // 避免重复计算
//     SX sin2_L_theta = sin(2 * L_theta);
//     SX cos2_L_theta = cos_L_theta * cos_L_theta;
//     SX sin2_L_theta_sq = sin_L_theta * sin_L_theta;
//
//     // 左腿x方向加速度
//     SX L_xdd_num = (5.82809 * L0_sq + 0.000206091) * L0
//         * L_theta_dot * L_theta_dot - sin_L_theta
//         - 28.9519 * L0_sq * sin2_L_theta
//         + 343.597 * L0_sq * L_T
//         + L0 * (11.4952 * L_T - 11.4952 * L_Tp) * cos_L_theta
//         + 0.0121501 * L_T;
//
//     SX L_xdd_den = 6.90856 * L0_sq * sin2_L_theta_sq
//         + L0_sq * cos2_L_theta
//         + 0.000244298;
//
//     // 保护分母
//     SX epsilon = 1e-6;
//     SX safe_L_xdd_den = fmax(L_xdd_den, epsilon);
//     SX L_x_ddot = L_xdd_num / safe_L_xdd_den;
//
//     // 左腿角度加速度（简化表达式）
//     // 避免计算复杂的sec_theta和tan_theta
//     // 使用近似：当角度较小时，tan(θ)≈θ，sec(θ)≈1
//
//     SX L_theta_ddot;
//     SX L_theta_small = fabs(L_theta) < 0.1745;  // 10度以内
//
//     // 小角度近似
//     SX L_theta_ddot_small = (9.93531 * L0 * L_theta
//         - 50.4216 * L0 * L_T
//         + 1.97239 * (L_Tp - L_T))
//         / (L0_sq * (0.144748 + L_theta * L_theta) + 0.0000353616);
//
//     // 大角度精确计算
//     SX sec_L_theta = 1.0 / cos_L_theta;
//     SX tan_L_theta = sin_L_theta / cos_L_theta;
//
//     SX L_thdd_num = sec_L_theta * (9.93531 * L0 * tan_L_theta
//         - 50.4216 * L0 * L_T
//         + (1.97239 * L_Tp - 1.97239 * L_T) * sec_L_theta)
//         - 0.855252 * L0_sq * L_theta_dot * L_theta_dot * tan_L_theta;
//
//     SX L_thdd_den = L0_sq * (tan_L_theta * tan_L_theta + 0.144748)
//         + 0.0000353616 * sec_L_theta * sec_L_theta;
//
//     SX safe_L_thdd_den = fmax(L_thdd_den, epsilon);
//     SX L_theta_ddot_large = L_thdd_num / safe_L_thdd_den;
//
//     // 选择近似或精确计算
//     L_theta_ddot = if_else(L_theta_small, L_theta_ddot_small, L_theta_ddot_large);
//
//     // 右腿类似处理...
//
//     // 车身角度加速度（简化）
//     SX phi_ddot = 2953.84 * (L_Tp + R_Tp) / 2;
//
//     // 合并状态导数
//     SX part1 = vertcat(L_theta_dot,
//         L_theta_ddot,
//         L_x_dot,
//         L_x_ddot,
//         R_theta_dot,
//         L_theta_ddot
//         );
//     SX part2 = vertcat(
//         R_x_dot,
//         L_x_ddot,      // 需要类似计算
//         phi_dot,
//         phi_ddot
//     );
//     SX x_dot = vertcat(part1, part2);
//
//     dynamics = Function("dynamics", {x, u, L0, L1}, {x_dot});
// }
//
//
// Function class_NMPC_casADi::getDiscreteDynamics() const
// {
//     SX x = SX::sym("x", n);
//     SX u = SX::sym("u", p);
//     SX L0 = SX::sym("L0");
//     SX L1 = SX::sym("L1");
//
//     // RK4 discretization
//     SX k1 = dynamics(std::vector<SX>{x, u, L0, L1})[0];
//     SX k2 = dynamics(std::vector<SX>{x + (dt / 2.0) * k1, u, L0, L1})[0];
//     SX k3 = dynamics(std::vector<SX>{x + (dt / 2.0) * k2, u, L0, L1})[0];
//     SX k4 = dynamics(std::vector<SX>{x + dt * k3, u, L0, L1})[0];
//
//     SX x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
//
//     return Function("discrete_dynamics", {x, u, L0, L1}, {x_next});
// }
//
// void class_NMPC_casADi::build_optimizer()
// {
//     // get discretization dynamics function
//     Function dis_dynamic = getDiscreteDynamics();
//
//     // definition optimizer variables
//     SX x = SX::sym("x", n, N + 1);
//     SX u = SX::sym("u", p, N);
//
//     // reference variables: initialize variables + complete reference trajectory + L0 + L1
//     int n_params = n + (N + 1) * n + 2;
//     SX P = SX::sym("p", n_params);
//
//     // get initialize state
//     SX x_init = P(Slice(0, n));
//
//     SX L0 = P(Slice(n_params - 2));
//     SX L1 = P(Slice(n_params - 1));
//
//     // cost function
//     SX J = 0;
//
//     // weigh matrix
//     DM Q = DM::diag({100.0, 1.0, 100.0, 10.0, 100.0, 1.0, 100.0, 10.0, 200.0, 1.0});
//     DM R = DM::diag({0.1, 0.1, 0.1, 0.1});
//     R = R * 100;
//     Q = Q * 100;
//
//     // constraints vectory
//     std::vector<SX> constraints;
//
//     // 1. initialize state constraints
//     constraints.push_back(x(Slice(), 0) - x_init);
//
//     // 2. dynamics constraints and calculation of cost function
//     for (int k = 0; k < N; k++)
//     {
//         SX x_k = x(Slice(), k);
//         SX u_k = u(Slice(), k);
//
//         // get current reference trajectory
//         SX x_ref_k = P(Slice(n + k * n, n + (k + 1) * n));
//
//         // cost function
//         J += mtimes((x_k - x_ref_k).T(), mtimes(casadi::SX(Q), (x_k - x_ref_k)));
//         J += mtimes(u_k.T(), mtimes(casadi::SX(R), u_k));
//
//         // dynamic constraints
//         SX x_next_dyn = dis_dynamic(std::vector<SX>{x_k, u_k, L0, L1})[0];
//         SX x_next = x(Slice(), k + 1);
//
//         constraints.push_back(x_next - x_next_dyn);
//     }
//
//     // final cost
//     SX x_final = x(Slice(), N);
//     SX x_ref_final = P(Slice(n + N * n, n + (N + 1) * n));
//     J += mtimes((x_final - x_ref_final).T(), mtimes(10.0 * casadi::SX(Q), (x_final - x_ref_final)));
//
//     // merge constraints
//     SX g = vertcat(constraints);
//
//     // merge optimized variables
//     SX opt = vertcat(
//         SX::reshape(x, (N + 1) * n, 1),
//         SX::reshape(u, N * p, 1)
//     );
//
//     // build NLP problem
//     SXDict nlp = {
//         {"x", opt},
//         {"f", J},
//         {"g", g},
//         {"p", P}
//     };
//
//     // set solver options
//     Dict opts;
//     opts["ipopt.print_level"] = 0;
//     opts["ipopt.tol"] = 1e-3;
//     opts["ipopt.max_iter"] = 1000;
//     // 添加数值稳定性选项
//     // opts["ipopt.theta_max_fact"] = 0.1;  // 限制步长
//     // opts["ipopt.bound_relax_factor"] = 1e-8;  // 边界松弛因子
//     // opts["ipopt.constr_viol_tol"] = 1e-6;  // 约束违反容忍度
//     // opts["ipopt.acceptable_tol"] = 1e-4;  // 可接受容忍度
//
//     opts["ipopt.print_level"] = 0;
//     opts["ipopt.tol"] = 1e-3;
//     opts["ipopt.max_iter"] = 100;
//
//     // 关键优化：使用有限内存的BFGS近似Hessian
//     opts["ipopt.hessian_approximation"] = "limited-memory";
//     opts["ipopt.limited_memory_max_history"] = 10;  // 存储的历史步数
//
//     // 禁用二阶导数计算，使用准牛顿法
//     opts["ipopt.hessian_constant"] = "no";
//
//     // 其他优化设置
//     opts["ipopt.warm_start_init_point"] = "yes";
//     opts["ipopt.mu_init"] = 1e-3;
//     opts["ipopt.bound_relax_factor"] = 1e-8;
//     opts["ipopt.max_cpu_time"] = 0.5;  // 限制最大计算时间
//
//     // 禁用线搜索（对非线性问题可能更快）
//     opts["ipopt.line_search_method"] = "cg-penalty";
//     opts["ipopt.accept_every_trial_step"] = "yes";
//
//     solver = nlpsol("solver", "ipopt", nlp, opts);
// }
//
// std::tuple<std::vector<double>, DM, DM> class_NMPC_casADi::solve(const std::vector<double>& x_init,
//                                              const std::vector<std::vector<double>>& reference_trajectory,
//                                              double L0, double L1) const
// {
//     // build reference vector
//     std::vector<double> P;
//
//     // 1. set initialize state
//     P.insert(P.end(), x_init.begin(), x_init.end());
//
//     // 2. complete reference trajectory
//     for (const auto& ref_state : reference_trajectory)
//     {
//         P.insert(P.end(), ref_state.begin(), ref_state.end());
//     }
//
//     // 3. L0 L1
//     P.push_back(L0);
//     P.push_back(L1);
//
//     // initialize guess
//     std::vector<double> x0;
//
//     // state guess: using the reference trajectory
//     // for (const auto& ref_state : reference_trajectory)
//     // {
//     //     x0.insert(x0.end(), ref_state.begin(), ref_state.end());
//     // }
//
//     for (int i = 0; i < N + 1; ++i)
//     {
//         x0.insert(x0.end(), x_init.begin(), x_init.end());
//     }
//     // for (int i = 0; i < N; ++i)
//     // {
//     //     x0.push_back(0.0);
//     // }
//
//     // control guess: zero control
//     for (int i = 0; i < N; ++i)
//     {
//         x0.push_back(0.0);
//         x0.push_back(0.0);
//         x0.push_back(0.0);
//         x0.push_back(0.0);
//     }
//
//     // set boundaries
//     std::vector<double> lb, ub, lba, uba;
//
//     // total number of variables
//     int n_vars = (N + 1) * n + N * p;
//     int n_constraints = n + N * n;
//
//     // state boundaries
//     for (int i = 0; i < N + 1; i++)
//     {
//         for (int j = 0; j < n; j++)
//         {
//             lb.push_back(x_min[j]);
//             ub.push_back(x_max[j]);
//         }
//     }
//
//     // control boundaries
//     for (int i = 0; i < N; i++)
//     {
//         for (int j = 0; j < p; j++)
//         {
//             lb.push_back(u_min[j]);
//             ub.push_back(u_max[j]);
//         }
//     }
//
//     // constraints boundaries
//     for (int i = 0; i < n_constraints; i++)
//     {
//         lba.push_back(0.0);
//         uba.push_back(0.0);
//     }
//
//     // solve
//     DMDict arg = {
//         {"x0", x0},
//         {"lbx", lb},
//         {"ubx", ub},
//         {"lbg", lba},
//         {"ubg", uba},
//         {"p", P}
//     };
//
//     DMDict res = solver(arg);
//
//     // extract
//     DM opt = res.at("x");
//
//     // the state trajectory
//     DM X_opt = reshape(opt(Slice(0, n * (N + 1))), n, N + 1);
//
//     // 提取控制轨迹
//     DM U_opt = reshape(opt(Slice(n * (N + 1), n * (N + 1) + p * N)), p, N);
//
//     // 返回第一个控制输入和完整轨迹
//     std::vector<double> u_opt = {};
//     u_opt[0] = 0;
//     u_opt[1] = 0;
//     u_opt[2] = 0;
//     u_opt[3] = 0;
//     // u_opt[0] = static_cast<double>(U_opt(0, 0));
//     // u_opt[1] = static_cast<double>(U_opt(1, 0));
//     // u_opt[2] = static_cast<double>(U_opt(2, 0));
//     // u_opt[3] = static_cast<double>(U_opt(3, 0));
//
//     return std::make_tuple(u_opt, X_opt, U_opt);
// }
//
//
// std::tuple<std::vector<double>, DM, DM> class_NMPC_casADi::solve1(const std::vector<double>& x_init,
//                                              const std::vector<std::vector<double>>& reference_trajectory,
//                                              double L0, double L1, const std::vector<double>& guess) const
// {
//     // // build reference vector
//     // std::vector<double> P;
//     //
//     // // 1. set initialize state
//     // P.insert(P.end(), x_init.begin(), x_init.end());
//     //
//     // // 2. complete reference trajectory
//     // for (const auto& ref_state : reference_trajectory)
//     // {
//     //     P.insert(P.end(), ref_state.begin(), ref_state.end());
//     // }
//     //
//     // // 3. L0 L1
//     // P.push_back(L0);
//     // P.push_back(L1);
//     //
//     // // initialize guess
//     // std::vector<double> x0;
//     //
//     // // state guess: using the reference trajectory
//     // // for (const auto& ref_state : reference_trajectory)
//     // // {
//     // //     x0.insert(x0.end(), ref_state.begin(), ref_state.end());
//     // // }
//     //
//     // if (guess.empty())
//     // {
//     //     // 如果没有提供猜测，使用初始状态
//     //     for (int i = 0; i < N + 1; ++i)
//     //     {
//     //         x0.insert(x0.end(), x_init.begin(), x_init.end());
//     //     }
//     //     // control guess: zero control
//     //     for (int i = 0; i < N; ++i)
//     //     {
//     //         x0.push_back(0.0);
//     //         x0.push_back(0.0);
//     //         x0.push_back(0.0);
//     //         x0.push_back(0.0);
//     //     }
//     // }
//     // else
//     // {
//     //     x0 = guess;
//     // }
//     //
//     //
//     // // set boundaries
//     // std::vector<double> lb, ub, lba, uba;
//     //
//     // // total number of variables
//     // int n_vars = (N + 1) * n + N * p;
//     // int n_constraints = n + N * n;
//     //
//     // // state boundaries
//     // for (int i = 0; i < N + 1; i++)
//     // {
//     //     for (int j = 0; j < n; j++)
//     //     {
//     //         lb.push_back(x_min[j]);
//     //         ub.push_back(x_max[j]);
//     //     }
//     // }
//     //
//     // // control boundaries
//     // for (int i = 0; i < N; i++)
//     // {
//     //     for (int j = 0; j < p; j++)
//     //     {
//     //         lb.push_back(u_min[j]);
//     //         ub.push_back(u_max[j]);
//     //     }
//     // }
//     //
//     // // constraints boundaries
//     // for (int i = 0; i < n_constraints; i++)
//     // {
//     //     lba.push_back(0.0);
//     //     uba.push_back(0.0);
//     // }
//     //
//     // // solve
//     // DMDict arg = {
//     //     {"x0", x0},
//     //     {"lbx", lb},
//     //     {"ubx", ub},
//     //     {"lbg", lba},
//     //     {"ubg", uba},
//     //     {"p", P}
//     // };
//     //
//     // DMDict res = solver(arg);
//     //
//     // // extract
//     // DM opt = res.at("x");
//     //
//     // // the state trajectory
//     // DM X_opt = reshape(opt(Slice(0, n * (N + 1))), n, N + 1);
//     //
//     // // 提取控制轨迹
//     // DM U_opt = reshape(opt(Slice(n * (N + 1), n * (N + 1) + p * N)), p, N);
//     //
//     // std::vector<double>u_opt(p);
//     // // double u_opt = static_cast<double>(U_opt(0,0));
//     // u_opt[0] = static_cast<double>(U_opt(0,0));
//     // u_opt[1] = static_cast<double>(U_opt(1,0));
//     // u_opt[2] = static_cast<double>(U_opt(2,0));
//     // u_opt[3] = static_cast<double>(U_opt(3,0));
//     //
//     //
//     // return std::make_tuple(u_opt, X_opt, U_opt);
//
//
//     // 1. 预分配内存，避免动态分配
//     std::vector<double> P;
//     P.reserve(n + (N + 1) * n + 2);  // 预分配内存
//
//     // 添加初始状态
//     P.insert(P.end(), x_init.begin(), x_init.end());
//
//     // 2. 使用emplace_back避免拷贝
//     for (const auto& ref_state : reference_trajectory) {
//         P.insert(P.end(), ref_state.begin(), ref_state.end());
//     }
//
//     // 3. 直接添加L0, L1
//     P.push_back(L0);
//     P.push_back(L1);
//
//     // 4. 优化初始猜测
//     std::vector<double> x0;
//     if (guess.empty()) {
//         x0.resize((N + 1) * n + N * p, 0.0);  // 一次性分配
//
//         // 使用memcpy批量填充
//         for (int i = 0; i < N + 1; ++i) {
//             std::memcpy(&x0[i * n], x_init.data(), n * sizeof(double));
//         }
//         // 控制部分已经是0，无需填充
//     } else {
//         x0 = guess;
//     }
//
//     // 5. 边界约束预计算
//     static std::vector<double> lb, ub, lba, uba;
//     static bool first_call = true;
//
//     if (first_call) {
//         lb.reserve((N + 1) * n + N * p);
//         ub.reserve((N + 1) * n + N * p);
//
//         // 状态边界
//         for (int i = 0; i < N + 1; i++) {
//             lb.insert(lb.end(), x_min.begin(), x_min.end());
//             ub.insert(ub.end(), x_max.begin(), x_max.end());
//         }
//
//         // 控制边界
//         for (int i = 0; i < N; i++) {
//             lb.insert(lb.end(), u_min.begin(), u_min.end());
//             ub.insert(ub.end(), u_max.begin(), u_max.end());
//         }
//
//         // 等式约束边界
//         int n_constraints = n + N * n;
//         lba.assign(n_constraints, 0.0);
//         uba.assign(n_constraints, 0.0);
//
//         first_call = false;
//     }
//
//
//     // 重新创建求解器（或者可以缓存）
//     // 注意：这需要修改类设计，将solver作为mutable
//
//     // 7. 使用热启动
//     DMDict arg = {
//         {"x0", x0},
//         {"lbx", lb},
//         {"ubx", ub},
//         {"lbg", lba},
//         {"ubg", uba},
//         {"p", P}
//     };
//     // 这是一个极其有用的调试代码
//     try {
//         // 1. 获取输入参数
//         std::vector<double> x0_vec = static_cast<std::vector<double>>(arg["x0"]);
//         std::vector<double> p_vec = static_cast<std::vector<double>>(arg["p"]);
//
//         // 2. 调用你的 dynamics 函数测试一下 (手动计算第一步)
//         // 假设前 n 个是 x_init, 后 p 个是 u_init
//         std::vector<double> x_test(x0_vec.begin(), x0_vec.begin() + n);
//         std::vector<double> u_test(p, 0.0);
//
//         // 注意：你需要把 discrete_dynamics 保存为类成员才能在这里调用
//         // 或者简单地打印 x_init 看看有没有 NaN
//         std::cout << "Debug x_init: ";
//         for(auto v : x_init) std::cout << v << " ";
//         std::cout << "\nDebug L0: " << L0 << " L1: " << L1 << std::endl;
//
//     } catch (std::exception& e) {
//         std::cout << "Debug print error: " << e.what() << std::endl;
//     }
//
//     try {
//         DMDict res = solver(arg);
//         DM opt = res.at("x");
//
//         // 提取结果
//         DM X_opt = reshape(opt(Slice(0, n * (N + 1))), n, N + 1);
//         DM U_opt = reshape(opt(Slice(n * (N + 1), n * (N + 1) + p * N)), p, N);
//
//         std::vector<double> u_opt(p);
//         u_opt[0] = static_cast<double>(U_opt(0, 0));
//         u_opt[1] = static_cast<double>(U_opt(1, 0));
//         u_opt[2] = static_cast<double>(U_opt(2, 0));
//         u_opt[3] = static_cast<double>(U_opt(3, 0));
//
//         return std::make_tuple(u_opt, X_opt, U_opt);
//     } catch (std::exception& e) {
//         // 求解失败，返回保守控制
//         std::vector<double> u_opt(p, 0.0);
//         return std::make_tuple(u_opt, DM(), DM());
//     }
// }
//
//


/**
@file           : NMPC_casADi.cpp
@author         : Chen Haoran
*/

#include "NMPC_casADi.h"

using namespace casadi;

class_NMPC_casADi::class_NMPC_casADi()
{
    // initialize variables
    n = 6;
    p = 4;
    N = 10;
    dt = 0.03; // 建议检查 dt，0.1可能对于平衡控制太大了，通常 0.01-0.05

    // 状态约束
    x_min = {-1.0, -0.5, -0.5, -0.5, -inf, -2.0};
    x_max = { 1.0,  0.5,  0.5,  0.5,  inf,  2.0,};

    // 控制约束
    u_min = {-0.5, -1.0, -0.5, -0.5};
    u_max = {0.5, 1.5, 0.5, 0.5};

    build_dynamics();
    build_optimizer();
}

void class_NMPC_casADi::build_dynamics()
{
    // =========================================================================
    // 关键修改：使用 SX 替代 MX
    // =========================================================================
    SX x = SX::sym("x", n);
    SX u = SX::sym("u", p);
    SX L0 = SX::sym("L0");
    SX L1 = SX::sym("L1");

    // 提取状态
    SX L_theta = x(0);
    SX L_theta_dot = x(1);
    SX R_theta = x(2);
    SX R_theta_dot = x(3);
    SX X = x(4);
    SX X_dot = x(5);

    SX L_T = u(0);
    SX L_Tp = u(1);
    SX R_T = u(2);
    SX R_Tp = u(3);

    // 辅助变量
    double eps = 1e-6; // 防止分母为0的安全小量

    // L0, L1 平方保护
    SX L0_sq = L0 * L0 + eps;
    SX L1_sq = L1 * L1 + eps;

    // 三角函数
    SX sin_L_theta = sin(L_theta);
    SX cos_L_theta = cos(L_theta);
    SX sin_2L_theta = sin(2 * L_theta);

    // ======================== L_x_ddot 计算 ========================
    SX Xdd_num = (5.82809 * L0 * L0 + 0.000206091) * L0 * L_theta_dot * L_theta_dot
        - sin_L_theta
        - 28.9519 * L0 * L0 * sin_2L_theta
        + 343.597 * L0 * L0 * L_T
        + L0 * (11.4952 * L_T - 11.4952 * L_Tp) * cos_L_theta
        + 0.0121501 * L_T;

    SX Xdd_den = 6.90856 * L0 * L0 * sin_L_theta * sin_L_theta
        + L0 * L0 * cos_L_theta * cos_L_theta
        + 0.000244298;

    // 【防NaN修改】分母加 eps
    SX X_ddot = Xdd_num / (Xdd_den + eps);

    // ======================== L_theta_ddot 计算 ========================
    // 使用 if_else 处理小角度奇点，且全域加 eps 保护

    // 1. 小角度近似公式
    // SX L_theta_ddot_small = (9.93531 * L0 * L_theta
    //         - 50.4216 * L0 * L_T
    //         + 1.97239 * (L_Tp - L_T))
    //     / (L0_sq * (0.144748 + L_theta * L_theta) + 0.0000353616 + eps);

    // 2. 大角度精确公式
    // 防止 cos(theta) = 0
    SX safe_cos_L = cos_L_theta + 1e-9 * sign(cos_L_theta);
    SX sec_L_theta = 1.0 / safe_cos_L;
    SX tan_L_theta = sin_L_theta / safe_cos_L;

    SX L_thdd_num = sec_L_theta * (9.93531 * L0 * tan_L_theta
            - 50.4216 * L0 * L_T
            + (1.97239 * L_Tp - 1.97239 * L_T) * sec_L_theta)
        - 0.855252 * L0_sq * L_theta_dot * L_theta_dot * tan_L_theta;

    SX L_thdd_den = L0_sq * (tan_L_theta * tan_L_theta + 0.144748)
        + 0.0000353616 * sec_L_theta * sec_L_theta;

    SX L_theta_ddot = L_thdd_num / (L_thdd_den + eps);

    // 混合逻辑：角度小于 10度 (0.1745 rad) 用近似，否则用精确
    // SX L_theta_ddot = if_else(fabs(L_theta) < 0.1745, L_theta_ddot_small, L_theta_ddot_large);

    // SX sec_theta = 1.0 / cos(L_theta);
    // SX tan_theta = sin(L_theta) / cos(L_theta);
    //
    // SX thdd_num = sec_theta * (9.93531 * L0 * tan_theta
    //         - 50.4216 * L0 * L_T
    //         + (1.97239 * L_Tp - 1.97239 * L_T) * sec_theta)
    //     - 0.855252 * pow(L0, 2) * pow(L_theta_dot, 2) * tan_theta;
    //
    // SX thdd_den = pow(L0, 2) * (1.0 * pow(tan_theta, 2) + 0.144748)
    //     + 0.0000353616 * pow(sec_theta, 2);
    //
    // SX L_theta_ddot = thdd_num / thdd_den;

    // ======================== R_x_ddot, R_theta_ddot (省略，逻辑同上，为了简洁直接给结果) ========================
    // SX R_sec_theta = 1.0 / cos(R_theta);
    // SX R_tan_theta = sin(R_theta) / cos(R_theta);
    //
    // SX R_thdd_num = R_sec_theta * (9.93531 * L1 * R_tan_theta
    //         - 50.4216 * L1 * R_T
    //         + (1.97239 * R_Tp - 1.97239 * R_T) * R_sec_theta)
    //     - 0.855252 * pow(L1, 2) * pow(R_theta_dot, 2) * R_tan_theta;
    //
    // SX R_thdd_den = pow(L1, 2) * (1.0 * pow(R_tan_theta, 2) + 0.144748)
    //     + 0.0000353616 * pow(R_sec_theta, 2);
    //
    // SX R_theta_ddot = R_thdd_num / R_thdd_den;
    SX R_theta_ddot = 0;


    // 组合 x_dot
    // 注意顺序必须与 x 定义一致: [L_th, L_th_d, L_x, L_x_d, R_th, R_th_d, R_x, R_x_d, phi, phi_d]
    SX x_dot = vertcat(
        L_theta_dot,
        L_theta_ddot,
        R_theta_dot,
        R_theta_ddot,
        X_dot,
        X_ddot
    );

    dynamics = Function("dynamics", {x, u, L0, L1}, {x_dot});
}

Function class_NMPC_casADi::getDiscreteDynamics() const
{
    // SX 版本的 RK4
    SX x = SX::sym("x", n);
    SX u = SX::sym("u", p);
    SX L0 = SX::sym("L0");
    SX L1 = SX::sym("L1");

    // 调用 dynamics 时，输入是 vector<SX>
    SX k1 = dynamics(std::vector<SX>{x, u, L0, L1})[0];
    SX k2 = dynamics(std::vector<SX>{x + (dt / 2.0) * k1, u, L0, L1})[0];
    SX k3 = dynamics(std::vector<SX>{x + (dt / 2.0) * k2, u, L0, L1})[0];
    SX k4 = dynamics(std::vector<SX>{x + dt * k3, u, L0, L1})[0];

    SX x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    return Function("discrete_dynamics", {x, u, L0, L1}, {x_next});
}

void class_NMPC_casADi::build_optimizer()
{
    Function dis_dynamic = getDiscreteDynamics();

    // 优化变量 (SX)
    // x: 状态轨迹 [x0, x1, ..., xN]
    // u: 控制轨迹 [u0, u1, ..., uN-1]
    SX x = SX::sym("x", n, N + 1);
    SX u = SX::sym("u", p, N);

    // 参数 P: [x_init, ref_traj(N+1), L0, L1]
    // 总长度: n + (N+1)*n + 2
    int n_params = n + (N + 1) * n + 2;
    SX P = SX::sym("P", n_params);

    // 提取参数
    SX x_init = P(Slice(0, n));
    SX L0 = P(n_params - 2); // 直接索引，无需 Slice(idx, idx+1)
    SX L1 = P(n_params - 1);

    SX J = 0; // 代价函数
    std::vector<SX> g_vec; // 约束向量

    // 权重矩阵 (SX 矩阵)
    // 使用 SX::diag 生成对角阵
    double a = 100;
    std::vector<double> Q_diag = {
        2.0, 0.2 , 2.0 , 0.2 , 1.0 , 1.0
    };
    SX Q = SX::diag(DM(Q_diag)); // DM 转 SX

    std::vector<double> R_diag = {10 , 10 , 10 , 10 };
    SX R = SX::diag(DM(R_diag));

    // 1. 初始状态约束 (Row 0 ~ 9)
    // 确保 x 的第一列等于初始状态
    g_vec.push_back(x(Slice(), 0) - x_init);

    // 2. 动力学约束和代价函数
    for (int k = 0; k < N; ++k)
    {
        SX x_k = x(Slice(), k);
        SX u_k = u(Slice(), k);
        SX x_next_var = x(Slice(), k + 1); // 优化变量中的下一刻

        // 参考状态
        SX x_ref_k = P(Slice(n + k * n, n + (k + 1) * n));

        // Cost Calculation
        SX x_err = x_k - x_ref_k;
        J += SX::mtimes({x_err.T(), Q, x_err}); // SX 链式乘法
        J += SX::mtimes({u_k.T(), R, u_k});

        // Dynamic Constraints
        // 调用 RK4 函数
        SX x_next_calc = dis_dynamic(std::vector<SX>{x_k, u_k, L0, L1})[0];

        g_vec.push_back(x_next_var - x_next_calc);
    }

    // 终端代价
    SX x_final = x(Slice(), N);
    SX x_ref_final = P(Slice(n + N * n, n + (N + 1) * n));
    SX x_err_final = x_final - x_ref_final;
    J += SX::mtimes({x_err_final.T(), Q, x_err_final});

    // 展平优化变量
    // reshape(col_vector) -> vector
    SX opt_vars = vertcat(
        reshape(x, n * (N + 1), 1),
        reshape(u, p * N, 1)
    );

    SX g = vertcat(g_vec);

    // 构建 NLP
    SXDict nlp = {
        {"x", opt_vars},
        {"f", J},
        {"g", g},
        {"p", P}
    };

    // 求解器选项 (关键优化)
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["ipopt.sb"] = "yes"; // 抑制横幅
    opts["ipopt.max_iter"] = 100; // 实时控制不需要太多迭代
    opts["ipopt.tol"] = 1e-3;
    opts["ipopt.acceptable_tol"] = 1e-2;
    opts["ipopt.warm_start_init_point"] = "yes"; // 开启热启动

    // SX 特有的，允许更快的计算
    opts["expand"] = true; // 将计算图展开为标量运算，极大提升速度

    solver = nlpsol("solver", "ipopt", nlp, opts);
}

// Solve1 函数基本保持不变，因为接口使用 DM
std::tuple<std::vector<double>, DM, DM> class_NMPC_casADi::solve1(
    const std::vector<double>& x_init,
    const std::vector<std::vector<double>>& reference_trajectory,
    double L0, double L1, const std::vector<double>& guess) const
{
    // ------------------------------------
    // 1. 输入校验与保护 (防止 NaN)
    // ------------------------------------
    if (reference_trajectory.size() != N + 1)
    {
        std::cerr << "[NMPC] Error: Ref Traj size " << reference_trajectory.size() << " != " << N + 1 << std::endl;
        return {std::vector<double>(p, 0), DM(), DM()};
    }

    double safe_L0 = L0;
    double safe_L1 = L1;

    // ------------------------------------
    // 2. 构建参数 P
    // ------------------------------------
    std::vector<double> P_vec;
    P_vec.reserve(n + (N + 1) * n + 2);

    // Initial state
    P_vec.insert(P_vec.end(), x_init.begin(), x_init.end());
    // Ref trajectory
    for (const auto& ref : reference_trajectory)
    {
        P_vec.insert(P_vec.end(), ref.begin(), ref.end());
    }
    // Parameters
    P_vec.push_back(safe_L0);
    P_vec.push_back(safe_L1);

    // ------------------------------------
    // 3. 构建初值 Guess (防止 Row 0 NaN)
    // ------------------------------------
    std::vector<double> x0_vec;
    x0_vec.reserve((N + 1) * n + N * p);

    if (guess.empty())
    {
        // 使用 x_init 填充整个预测时域作为状态初值 (比全0更安全)
        for (int i = 0; i < N + 1; ++i)
        {
            x0_vec.insert(x0_vec.end(), x_init.begin(), x_init.end());
        }
        // 控制量初值设为0
        x0_vec.resize((N + 1) * n + N * p, 0.0);
    }
    else
    {
        x0_vec = guess;
    }

    // ------------------------------------
    // 4. 构建边界 (使用 static 缓存优化)
    // ------------------------------------
    // 为了线程安全，这里暂时不用 static，如果单线程跑可以加 static
    std::vector<double> lbx, ubx, lbg, ubg;

    // 状态边界
    for (int i = 0; i < N + 1; ++i)
    {
        lbx.insert(lbx.end(), x_min.begin(), x_min.end());
        ubx.insert(ubx.end(), x_max.begin(), x_max.end());
    }
    // 控制边界
    for (int i = 0; i < N; ++i)
    {
        lbx.insert(lbx.end(), u_min.begin(), u_min.end());
        ubx.insert(ubx.end(), u_max.begin(), u_max.end());
    }
    // 约束边界 (全0 表示等式约束)
    lbg.assign(n + N * n, 0.0);
    ubg.assign(n + N * n, 0.0);

    // ------------------------------------
    // 5. 调用求解器
    // ------------------------------------
    DMDict arg = {
        {"x0", x0_vec},
        {"lbx", lbx},
        {"ubx", ubx},
        {"lbg", lbg},
        {"ubg", ubg},
        {"p", P_vec}
    };

    try
    {
        DMDict res = solver(arg);

        // 提取结果
        DM opt_all = res.at("x");

        // 切片获取状态和控制 (注意索引计算)
        // 状态: 前 n*(N+1) 个
        DM X_opt = reshape(opt_all(Slice(0, n * (N + 1))), n, N + 1);

        // 控制: 后 p*N 个
        DM U_opt = reshape(opt_all(Slice(n * (N + 1), n * (N + 1) + p * N)), p, N);

        // 返回第一个控制量
        std::vector<double> u_res(p);
        u_res[0] = static_cast<double>(U_opt(0, 0));
        u_res[1] = static_cast<double>(U_opt(1, 0));
        u_res[2] = static_cast<double>(U_opt(2, 0));
        u_res[3] = static_cast<double>(U_opt(3, 0));

        return {u_res, X_opt, U_opt};
    }
    catch (std::exception& e)
    {
        std::cerr << "[NMPC] Solver failed: " << e.what() << std::endl;
        return {std::vector<double>(p, 0), DM(), DM()};
    }
}

// 占位函数，为兼容
std::tuple<std::vector<double>, DM, DM> class_NMPC_casADi::solve(
    const std::vector<double>& x_init,
    const std::vector<std::vector<double>>& reference_trajectory,
    double L0, double L1) const
{
    return solve1(x_init, reference_trajectory, L0, L1, {});
}
