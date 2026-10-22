/**
  ******************************************************************************
  * @file           : NMPC.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/20
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "NMPC.h"
#include "Algorithm/system_dynamic/phi_ddot.h"
#include "Algorithm/system_dynamic/theta_ddot.h"
#include "Algorithm/system_dynamic/x_ddot.h"
#include "Algorithm/VMC/lqr_k_calc.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/
ADdouble theta_ddot_ad(ADdouble L0, ADdouble T, ADdouble Tp, ADdouble theta, ADdouble theta1);
ADdouble x_ddot_ad(ADdouble L0, ADdouble T, ADdouble Tp, ADdouble theta, ADdouble theta1);
ADdouble phi_ddot_ad(ADdouble Tp);
/* Function ------------------------------------------------------------------*/

using namespace Eigen;

class_NMPC::class_NMPC(int n, int p, int y, int N, float dt) : n(n), p(p), y(y), N(N), dt(dt)
{
    // 初始化矩阵
    x = VectorXd::Zero(n);
    x_ref = VectorXd::Zero((N + 1) * n);
    u_ref = VectorXd::Zero(N * p);
    n_vars_ = N * (n + p) + n;
    n_eq_constraints_ = (N + 1) * n;
    nV = n_vars_;
    nC = n_eq_constraints_ + n_ineq_constraints_;


    A = MatrixXd::Identity(n, n);
    B = MatrixXd::Zero(n, p);
    Q = MatrixXd::Zero(n, n);
    R = MatrixXd::Zero(p, p);
    F = MatrixXd::Zero(n, n);

    x_min = VectorXd::Zero(n);
    x_max = VectorXd::Zero(n);
    u_min = VectorXd::Zero(p);
    u_max = VectorXd::Zero(p);

    H = MatrixXd::Zero(n_vars_, n_vars_);
    g = VectorXd::Zero(n_vars_);
    lb = VectorXd::Zero(n_vars_);
    ub = VectorXd::Zero(n_vars_);
    A_mat = MatrixXd::Zero(n_eq_constraints_ + n_ineq_constraints_, n_vars_);
    lba = VectorXd::Zero(n_eq_constraints_ + n_ineq_constraints_);
    uba = VectorXd::Zero(n_eq_constraints_ + n_ineq_constraints_);

    A_eq = MatrixXd::Zero(n_eq_constraints_, n_vars_);
    b_eq = VectorXd::Zero(n_eq_constraints_);
    A_ineq = MatrixXd::Identity(n_ineq_constraints_, n_vars_);
    b_ineq = VectorXd::Zero(n_ineq_constraints_);
}

SQPSolution class_NMPC::solve()
{
    // 1. initialize trajectory
    VectorXd current_trajectory = initialize_trajectory();
    VectorXd lambda_eq = VectorXd::Zero(n_eq_constraints_);
    VectorXd lambda_ineq = VectorXd::Zero(n_ineq_constraints_);

    // 2. SQP cycle
    for (int i = 0; i < max_sqp_iterations_; ++i)
    {
        // 2.1 linearization at the current trajectory
        auto linearization = linearize_at_trajectory(current_trajectory);

        // 2.2 constructing qp sub problem
        build_QP_subproblem(linearization, current_trajectory);

        // 2.3 use qpOASES solve qp
        auto [delta_traj, lambda_eq_new, lambda_ineq_new] = solve_QP_with_qpOASES();

        // 2.4 line search
        double alpha = line_search(current_trajectory, delta_traj);

        // 2.5 update trajectory
        VectorXd new_trajectory = current_trajectory + alpha * delta_traj;

        // 2.6 update Hessian (BFGS)
        // H = update_hessian_BFGS(current_trajectory, new_trajectory,
        //                         lambda_eq_new, lambda_ineq_new);

        // 2.7 update multiplier
        lambda_eq = lambda_eq_new;
        lambda_ineq = lambda_ineq_new;

        // 2.8 check convergence
        if (check_convergence(current_trajectory, new_trajectory, delta_traj))
        {
            std::cout << "SQP converges in the " << i << " iteration" << std::endl;
            current_trajectory = new_trajectory;
            solution.converged = true;
            break;
        }

        current_trajectory = new_trajectory;
    }

    // 3. ready to return solution
    solution.trajectory = current_trajectory;
    solution.lambda_eq = lambda_eq;
    solution.lambda_ineq = lambda_ineq;
    solution.cost = compute_total_cost(current_trajectory);
    solution.iterations = max_sqp_iterations_;

    return solution;
}

VectorXd class_NMPC::initialize_trajectory() const
{
    VectorXd trajectory(n_vars_);

    // // simple initialization
    // for (int k = 0; k < N; ++k)
    // {
    //     // state segment
    //     trajectory.segment(k * (n + p), n) = x;
    //     // control segment
    //     trajectory.segment(k * (n + p) + n, p) = VectorXd::Zero(p);
    // }
    //
    // trajectory.segment(N * (n + p), n) = x;

    // 从当前状态开始
    VectorXd current_state = x;

    for (int k = 0; k < N; ++k)
    {
        // 设置当前状态
        trajectory.segment(k * (n + p), n) = current_state;

        // 计算维持当前状态所需的控制输入（近似）
        VectorXd u_k = compute_equilibrium_control(current_state);

        // 设置控制输入
        trajectory.segment(k * (n + p) + n, p) = u_k;

        // 前向传播到下一个状态（使用简化的动力学）
        if (k < N - 1)
        {
            current_state = current_state + dt * build_dynamics(current_state, u_k);
        }
    }

    // 终端状态
    trajectory.segment(N * (n + p), n) = current_state;

    return trajectory;
}

LinearizationResult class_NMPC::linearize_at_trajectory(const VectorXd& trajectory) const
{
    LinearizationResult result;

    // for (int k = 0; k < N; ++k)
    // {
    //     constexpr double epsilon = 1e-6;
    //     // get current state and control vector
    //     VectorXd x_k = get_state_from_trajectory(trajectory, k);
    //     VectorXd u_k = get_control_from_trajectory(trajectory, k);
    //
    //     // build current dynamics
    //     VectorXd f_current = build_dynamics(x_k, u_k);
    //
    //     // numerical differential jacobian matrix Ak
    //     MatrixXd A_k = MatrixXd::Zero(n, n);
    //     for (int i = 0; i < n; ++i)
    //     {
    //         VectorXd x_plus = x_k;
    //         x_plus(i) += epsilon;
    //         VectorXd f_plus = build_dynamics(x_plus, u_k);
    //
    //         VectorXd x_sub = x_k;
    //         x_sub(i) -= epsilon;
    //         VectorXd f_sub = build_dynamics(x_sub, u_k);
    //
    //         A_k.col(i) = (f_plus - f_sub) / (2 * epsilon);
    //     }
    //
    //     // numerical differential jacobian matrix Bk
    //     MatrixXd B_k = MatrixXd::Zero(n, p);
    //     for (int i = 0; i < p; ++i)
    //     {
    //         VectorXd u_plus = u_k;
    //         u_plus(i) += epsilon;
    //         VectorXd f_plus = build_dynamics(x_k, u_plus);
    //
    //         VectorXd u_sub = u_k;
    //         u_sub(i) -= epsilon;
    //         VectorXd f_sub = build_dynamics(x_k, u_sub);
    //
    //         B_k.col(i) = (f_plus - f_sub) / (2 * epsilon);
    //     }
    //
    //     result.A_k.push_back(A_k);
    //     result.B_k.push_back(B_k);
    //     result.f_k.push_back(f_current);
    // }


    for (int k = 0; k < N; ++k)
    {
        // 获取当前状态和控制
        VectorXd x_k = get_state_from_trajectory(trajectory, k);
        VectorXd u_k = get_control_from_trajectory(trajectory, k);

        // 使用CppAD计算雅可比矩阵
        MatrixXd A_k, B_k;
        linearize_with_cppad(x_k, u_k, A_k, B_k);

        // 计算当前动力学（用于残差项）
        VectorXd f_current = build_dynamics(x_k, u_k);

        result.A_k.push_back(A_k);
        result.B_k.push_back(B_k);
        result.f_k.push_back(f_current);
    }

    // 终端状态的线性化（只需要A矩阵）
    VectorXd x_N = get_state_from_trajectory(trajectory, N);
    VectorXd u_last = get_control_from_trajectory(trajectory, N-1); // 使用最后一个控制

    MatrixXd A_N, B_dummy;
    linearize_with_cppad(x_N, u_last, A_N, B_dummy);
    result.A_k.push_back(A_N);

    return result;
}

void class_NMPC::linearize_with_cppad(const VectorXd& x, const VectorXd& u, MatrixXd& A, MatrixXd& B) const
{
    // 1. 创建独立变量
    ADVector xu_ad(n + p);
    xu_ad.head(n) = x.cast<ADdouble>();
    xu_ad.tail(p) = u.cast<ADdouble>();

    CppAD::Independent(xu_ad);

    // 2. 拆分状态和控制
    ADVector x_ad = xu_ad.head(n);
    ADVector u_ad = xu_ad.tail(p);

    // 3. 计算动力学
    ADVector y_ad = build_dynamics_ad(x_ad, u_ad);

    // 4. 创建AD函数
    CppAD::ADFun<double> f;
    f.Dependent(xu_ad, y_ad);

    // 5. 计算Jacobian
    VectorXd xu = VectorXd::Zero(n + p);
    xu.head(n) = x;
    xu.tail(p) = u;

    VectorXd jacobian = f.Jacobian(xu);

    // 6. 重新组织为A和B矩阵
    A = Map<MatrixXd>(jacobian.data(), n, n + p).leftCols(n);
    B = Map<MatrixXd>(jacobian.data(), n, n + p).rightCols(p);
}

// wheel leg system dynamics
VectorXd class_NMPC::build_dynamics(const VectorXd& x, const VectorXd& u) const
{
    // state: [l_theta l_theta1 l_x l_x1 r_theta r_theta1 r_x r_x1 phi phi1 P]^T
    // state_dot: [l_theta1 l_theta2 l_x1 l_x2 r_theta1 r_theta2 r_x1 r_x2 phi phi1 P_dot]^T
    // control: [l_T l_Tp r_T r_Tp]^T

    VectorXd state_dot(n);

    double l_theta = x(0), l_theta1 = x(1);
    double l_x = x(2), l_x1 = x(3);
    double r_theta = x(4), r_theta1 = x(5);
    double r_x = x(6), r_x1 = x(7);
    double phi = x(8), phi1 = x(9);
    // double P = x(10);

    double l_T = u(0), l_Tp = u(1);
    double r_T = u(2), r_Tp = u(3);

    // obtain next quantity
    state_dot(0) = l_theta1;
    state_dot(1) = theta_ddot(L[0], l_T, l_Tp, l_theta, l_theta1) / 1000;
    state_dot(2) = l_x1;
    state_dot(3) = x_ddot(L[0], l_T, l_Tp, l_theta, l_theta1);
    state_dot(4) = r_theta1;
    state_dot(5) = theta_ddot(L[1], r_T, r_Tp, r_theta, r_theta1) / 1000;
    state_dot(6) = r_x1;
    state_dot(7) = x_ddot(L[1], r_T, r_Tp, r_theta, r_theta1);
    state_dot(8) = phi1;
    state_dot(9) = phi_ddot((l_Tp + r_Tp) / 2);


    // return temp vector
    return state_dot;
}

// CppAD types build dynamics
ADVector class_NMPC::build_dynamics_ad(const ADVector& x_ad, const ADVector& u_ad) const
{
    ADVector x_dot(n);

    // get state variables
    ADdouble l_theta = x_ad(0), l_theta1 = x_ad(1);
    ADdouble l_x = x_ad(2), l_x1 = x_ad(3);
    ADdouble r_theta = x_ad(4), r_theta1 = x_ad(5);
    ADdouble r_x = x_ad(6), r_x1 = x_ad(7);
    ADdouble phi = x_ad(8), phi1 = x_ad(9);

    // get control variables
    ADdouble l_T = u_ad(0), l_Tp = u_ad(1);
    ADdouble r_T = u_ad(2), r_Tp = u_ad(3);

    // Use ADdouble type calculating
    x_dot(0) = l_theta1;
    x_dot(1) = theta_ddot_ad(ADdouble(L[0]), l_T, l_Tp, l_theta, l_theta1);
    x_dot(2) = l_x1;
    x_dot(3) = x_ddot_ad(ADdouble(L[0]), l_T, l_Tp, l_theta, l_theta1);
    x_dot(4) = r_theta1;
    x_dot(5) = theta_ddot_ad(ADdouble(L[1]), r_T, r_Tp, r_theta, r_theta1);
    x_dot(6) = r_x1;
    x_dot(7) = x_ddot_ad(ADdouble(L[1]), r_T, r_Tp, r_theta, r_theta1);
    x_dot(8) = phi1;
    x_dot(9) = phi_ddot_ad((l_Tp + r_Tp) / ADdouble(2.0));

    return x_dot;
}

// 构建QP子问题
void class_NMPC::build_QP_subproblem(const LinearizationResult& lin, const VectorXd& current_traj)
{
    // 1. Hessian matrix, using gaussian-newton approximation
    build_gauss_newton_hessian();

    // 2. Gradient vector
    build_gradient(current_traj);

    // 3. equality constraint
    build_equality_constraints(lin, current_traj);

    // 4. inequality constraint
    build_inequality_constraints();

    // 5. variable boundary
    build_variable_bounds();
}

// gaussian-newton approximation
void class_NMPC::build_gauss_newton_hessian()
{
    // simplify the implementation of using control weight matrix
    H = MatrixXd::Zero(n_vars_, n_vars_);
    for (int k = 0; k < N; ++k)
    {
        // weight matrix increase gain
        double weight = pow(static_cast<double>(k) / N, c);
        weight = 1;

        for (int i = 0; i < n; ++i)
        {
            H(k * (n + p) + i, k * (n + p) + i) = weight * Q(i, i);
        }
        for (int i = 0; i < p; ++i)
        {
            H(k * (n + p) + n + i, k * (n + p) + n + i) = weight * R(i, i);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        H(N * (n + p) + i, N * (n + p) + i) = F(i, i);
    }
}

// constructing gradient vector
VectorXd class_NMPC::build_gradient(const VectorXd& current_traj)
{
    for (int k = 0; k < N; ++k)
    {
        double weight = pow(static_cast<double>(k) / N, c);
        weight = 1;

        // get variables from trajectory
        VectorXd x_k = get_state_from_trajectory(current_traj, k);
        VectorXd u_k = get_control_from_trajectory(current_traj, k);
        VectorXd x_tar = get_x_ref_from_trajectory(x_ref, k);
        VectorXd u_tar = get_u_ref_from_trajectory(u_ref, k);

        for (int i = 0; i < n; ++i)
        {
            g(k * (n + p) + i) = weight * Q(i, i) * (x_k(i) - x_tar(i));
        }

        for (int i = 0; i < p; ++i)
        {
            g(k * (n + p) + n + i) = weight * R(i, i) * (u_k(i) - u_tar(i));
        }
    }

    VectorXd x_k = get_state_from_trajectory(current_traj, N);
    VectorXd x_tar = get_x_ref_from_trajectory(x_ref, N);
    for (int i = 0; i < n; ++i)
    {
        g(N * (n + p) + i) = F(i, i) * (x_k(i) - x_tar(i));
    }

    return g;
}

// constructing equality constraint
void class_NMPC::build_equality_constraints(const LinearizationResult& lin, const VectorXd& current_traj)
{
    A_eq = MatrixXd::Zero(n_eq_constraints_, n_vars_);
    b_eq = VectorXd::Zero(n_eq_constraints_);

    // initialize constraint conditions
    // A_eq.block(0, 0, n, n) = MatrixXd::Identity(n, n);
    // b_eq.segment(0, n) = x;

    // dynamic constraint linearization
    for (int k = 0; k < N; ++k)
    {
        int row_start = (k) * n;
        int col_start_k = k * (n + p);
        int col_start_k1 = (k + 1) * (n + p);

        // 线性化约束: -f(x_k, u_k) + x_k1  = A_k δx_k + B_k δu_k - δx_{k+1}
        // A_eq.block(row_start, col_start_k, n, n) = lin.A_k[k] * dt;
        // A_eq.block(row_start, col_start_k + n, n, p) = lin.B_k[k] * dt;
        // A_eq.block(row_start, col_start_k1, n, n) = -MatrixXd::Identity(n, n);
        //
        // // 右侧项
        // VectorXd x_k1 = get_state_from_trajectory(current_traj, k + 1);
        //
        // // b_eq.segment(row_start, n) = -x_k1 + dt * lin.f_k[k];
        // // b_eq.segment(row_start, n) = VectorXd::Zero(n);
        //
        // VectorXd x_k = get_state_from_trajectory(current_traj, k);
        // VectorXd u_k = get_control_from_trajectory(current_traj, k);
        // // 计算残差项
        // VectorXd residual = dt * (lin.f_k[k] - lin.A_k[k] * x_k - lin.B_k[k] * u_k);
        // b_eq.segment(row_start, n) = residual - x_k1;
    }

    // // 2. 动力学约束 - 彻底重写
    // for (int k = 0; k < N; ++k)
    // {
    //     int row_start = (k) * n;
    //     int col_start_k = k * (n + p);
    //     int col_start_k1 = (k + 1) * (n + p);
    //
    //     // 获取当前轨迹点
    //     VectorXd x_k = get_state_from_trajectory(current_traj, k);
    //     VectorXd u_k = get_control_from_trajectory(current_traj, k);
    //     VectorXd x_k1_actual = get_state_from_trajectory(current_traj, k + 1);
    //
    //     // 使用前向欧拉离散化：x_{k+1} = x_k + dt * f(x_k, u_k)
    //     // 线性化后：x_{k+1} = x_k + dt * [f(x?_k, u?_k) + A_k (x_k - x?_k) + B_k (u_k - u?_k)]
    //
    //     // 约束矩阵
    //     A_eq.block(row_start, col_start_k, n, n) = MatrixXd::Identity(n, n) - dt * lin.A_k[k];
    //     A_eq.block(row_start, col_start_k + n, n, p) = -dt * lin.B_k[k];
    //     A_eq.block(row_start, col_start_k1, n, n) = -MatrixXd::Identity(n, n);
    //
    //     // 右侧项：-dt * [f(x?_k, u?_k) - A_k x?_k - B_k u?_k]
    //     VectorXd residual = -dt * (lin.f_k[k] - lin.A_k[k] * x_k - lin.B_k[k] * u_k);
    //     b_eq.segment(row_start, n) = residual;
    //
    //     // 调试：检查约束在当前轨迹点的残差
    //     VectorXd constraint_value =
    //         (MatrixXd::Identity(n, n) - dt * lin.A_k[k]) * x_k +
    //         (-dt * lin.B_k[k]) * u_k +
    //         (-MatrixXd::Identity(n, n)) * x_k1_actual -
    //         residual;
    //
    //     double constraint_error = constraint_value.norm();
    //     if (constraint_error > 1e-6) {
    //         std::cout << "dynamic constraint in k=" << k << " error: " << constraint_error << std::endl;
    //     }
    // }
}

// constructing inequality constraint
void class_NMPC::build_inequality_constraints()
{
}

// constructing variable bounds
void class_NMPC::build_variable_bounds()
{
    lb = VectorXd::Zero(n_vars_);
    ub = VectorXd::Zero(n_vars_);

    for (int k = 0; k < N; ++k)
    {
        // state min
        int state_idx = k * (n + p);
        lb.segment(state_idx, n) = x_min;
        ub.segment(state_idx, n) = x_max;

        // control min
        int control_idx = state_idx + n;
        lb.segment(control_idx, p) = u_min;
        ub.segment(control_idx, p) = u_max;
    }
    lb.segment(N * (n + p), n) = x_min;
    ub.segment(N * (n + p), n) = x_max;
}

// use qpOASES slove qp
std::tuple<VectorXd, VectorXd, VectorXd> class_NMPC::solve_QP_with_qpOASES()
{
    // Merge equality and inequality constraints
    int row_offset = 0;

    // equality constraint A_eq * z = b_eq → lbA = ubA = b_eq
    if (A_eq.rows() > 0)
    {
        Eigen::Map<MatrixXd>(A_mat.data() + row_offset * nV, A_eq.rows(), nV) = A_eq;
        for (int i = 0; i < b_eq.size(); ++i)
        {
            lba(row_offset + i) = b_eq(i) * 10.0;
            uba(row_offset + i) = b_eq(i) / 10.0;
        }
        row_offset += static_cast<int>(A_eq.rows());
    }

    // inequality constraints A_ineq * z <= b_ineq
    // if (A_ineq.rows() > 0)
    // {
    //     Eigen::Map<MatrixXd>(A_mat.data() + row_offset * nV, A_ineq.rows(), nV) = A_ineq;
    //     for (int i = 0; i < b_ineq.size(); ++i)
    //     {
    //         lba(row_offset + i) = -qpOASES::INFTY;
    //         uba(row_offset + i) = b_ineq(i);
    //     }
    // }

    // build and solve qp
    qpOASES::SQProblem solver(nV, nC);
    qpOASES::Options options;
    options.setToMPC();
    // options.printLevel = qpOASES::PL_LOW;
    solver.setOptions(options);

    int nWSR = 1000;
    qpOASES::returnValue status = solver.init(
        H.data(), g.data(),
        A_mat.data(),
        lb.data(), ub.data(),
        lba.data(), uba.data(),
        nWSR
    );

    // qpOASES::returnValue status = solver.init(
    //     H.data(), g.data(),
    //     nullptr,
    //     lb.data(), ub.data(),
    //     nullptr, nullptr,
    //     nWSR
    // );

    if (status == qpOASES::SUCCESSFUL_RETURN)
    {
        // get solution
        VectorXd solution(nV);
        solver.getPrimalSolution(solution.data());

        // get dual solution
        VectorXd dual_solution(nV + nC);
        solver.getDualSolution(dual_solution.data());

        VectorXd lambda_eq = VectorXd::Zero(A_eq.rows());
        VectorXd lambda_ineq = VectorXd::Zero(A_ineq.rows());

        lambda_eq = dual_solution.segment(nV, A_eq.rows());
        lambda_ineq = dual_solution.segment(nV + A_eq.rows(), A_ineq.rows());

        return {solution, lambda_eq, lambda_ineq};
    }
    else
    {
        // std::cerr << "Field to solve qp, return 0" << std::endl;
        return {
            VectorXd::Zero(nV),
            VectorXd::Zero(A_eq.rows()),
            VectorXd::Zero(A_ineq.rows())
        };
    }
}

// line search
double class_NMPC::line_search(const VectorXd& current_traj, const VectorXd& direction) const
{
    double alpha = 1.0;
    double c = 1e-4; // Armijo constant
    double rho = 0.5; // contraction factor
    int max_search = 10;

    double phi_0 = merit_function(current_traj);
    double dphi_0 = merit_function_directional_derivative(current_traj, direction);

    for (int i = 0; i < max_search; ++i)
    {
        VectorXd candidate = current_traj + alpha * direction;
        double phi_alpha = merit_function(candidate);

        // Armijo condition
        if (phi_alpha <= phi_0 + c * alpha * dphi_0)
        {
            return alpha;
        }
        alpha *= rho;
    }

    return alpha;
}

// BFGS update Hessian
MatrixXd class_NMPC::update_hessian_BFGS(const VectorXd& z_old, const VectorXd& z_new, const VectorXd& lambda_eq,
                                         const VectorXd& lambda_ineq)
{
    VectorXd s = z_new - z_old;

    // calculate gradient difference
    VectorXd y = gradient_lagrangian(z_new, lambda_eq, lambda_ineq) -
        gradient_lagrangian(z_old, lambda_eq, lambda_ineq);

    double sTy = s.dot(y);

    // avoid zero elimination
    if (std::abs(sTy) < 1e-10)
    {
        return H;
    }

    MatrixXd H_new = H;
    H_new += (y * y.transpose()) / sTy - (H * s) * (s.transpose() * H) / (s.transpose() * H * s);

    return H_new;
}

// check convergence conditions
bool class_NMPC::check_convergence(const VectorXd& z_old, const VectorXd& z_new, const VectorXd& step) const
{
    // 1. step is small
    if (step.norm() < tolerance_)
    {
        return true;
    }

    // 2. variable change very little
    if ((z_new - z_old).norm() < tolerance_)
    {
        return true;
    }

    return false;
}

// get state from trajectory
VectorXd class_NMPC::get_state_from_trajectory(const VectorXd& trajectory, int k) const
{
    return trajectory.segment(k * (n + p), n);
}

// get control form trajectory
VectorXd class_NMPC::get_control_from_trajectory(const VectorXd& trajectory, int k) const
{
    return trajectory.segment(k * (n + p) + n, p);
}

// get x_ref form trajectory
VectorXd class_NMPC::get_x_ref_from_trajectory(const VectorXd& trajectory, int k) const
{
    return trajectory.segment(k * n, n);
}

// get u_ref form trajectory
VectorXd class_NMPC::get_u_ref_from_trajectory(const VectorXd& trajectory, int k) const
{
    return trajectory.segment(k * p, p);
}

// merit function
double class_NMPC::merit_function(const VectorXd& trajectory) const
{
    return compute_total_cost(trajectory);
}

// merit function directional derivative
double class_NMPC::merit_function_directional_derivative(const VectorXd& trajectory, const VectorXd& direction) const
{
    // numerical direction calculation of derivative
    constexpr double epsilon = 1e-8;
    const double phi_0 = merit_function(trajectory);
    const double phi_eps = merit_function(trajectory + epsilon * direction);

    return (phi_eps - phi_0) / epsilon;
}

// compute total cost
double class_NMPC::compute_total_cost(const VectorXd& trajectory) const
{
    double cost = 0.0;

    for (int k = 0; k < N; ++k)
    {
        VectorXd x_k = get_state_from_trajectory(trajectory, k);
        VectorXd u_k = get_control_from_trajectory(trajectory, k);
        VectorXd x_tar = get_x_ref_from_trajectory(x_ref, k);
        VectorXd u_tar = get_u_ref_from_trajectory(u_ref, k);

        // status tracking cost
        cost += (x_k - x_tar).transpose() * Q * (x_k - x_tar);

        // control cost
        cost += (u_k - u_tar).transpose() * R * (u_k - u_tar);
    }

    return cost;
}


VectorXd class_NMPC::gradient_lagrangian(const VectorXd& z, const VectorXd& lambda_eq, const VectorXd& lambda_ineq)
{
    // simplify
    return build_gradient(z);
}


VectorXd class_NMPC::compute_equilibrium_control(const VectorXd& state) const
{
    // 基于你的系统，计算维持当前状态所需的控制输入
    // 这是一个近似，可以根据你的系统特性调整

    VectorXd u_eq = VectorXd::Zero(p);

    // 计算不同腿长下的LQR增益矩阵K
    float LQR_Tmp[12] = {0.0f};
    float LQR_K_L[2][6] = {};
    float LQR_K_R[2][6] = {};
    lqr_k_calc(L[0], LQR_Tmp);

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            LQR_K_L[j][i] = LQR_Tmp[i*2 + j];
        }
    }

    lqr_k_calc(L[1], LQR_Tmp);

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            LQR_K_R[j][i] = LQR_Tmp[i*2 + j];
        }
    }

    // LQR计算输出
    u_eq(0) = (LQR_K_L[0][0]*(state(0))
                +LQR_K_L[0][1]*(state(1))
                +LQR_K_L[0][2]*(state(2) - 0)
                +LQR_K_L[0][3]*(state(3) - 0)
                +LQR_K_L[0][4]*(state(8))
                +LQR_K_L[0][5]*(state(9)));

    u_eq(1) = (LQR_K_L[1][0]*(state(0))
                +LQR_K_L[1][1]*(state(1))
                +LQR_K_L[1][2]*(state(2))
                +LQR_K_L[1][3]*(state(3))
                +LQR_K_L[1][4]*(state(8))
                +LQR_K_L[1][5]*(state(9)));

    u_eq(2) = (LQR_K_L[1][0]*(state(4))
                +LQR_K_L[1][1]*(state(5))
                +LQR_K_L[1][2]*(state(6))
                +LQR_K_L[1][3]*(state(7))
                +LQR_K_L[1][4]*(state(8))
                +LQR_K_L[1][5]*(state(9)));

    u_eq(3) = (LQR_K_L[1][0]*(state(4))
                +LQR_K_L[1][1]*(state(5))
                +LQR_K_L[1][2]*(state(6))
                +LQR_K_L[1][3]*(state(7))
                +LQR_K_L[1][4]*(state(8))
                +LQR_K_L[1][5]*(state(9)));

    // 确保控制输入在边界内
    for (int i = 0; i < p; ++i)
    {
        u_eq(i) = std::max(u_min(i), std::min(u_max(i), u_eq(i)));
    }

    return u_eq;
}


/********************CppAD build dynamic********************/

ADdouble theta_ddot_ad(ADdouble L0, ADdouble T, ADdouble Tp, ADdouble theta, ADdouble theta1)
{
    ADdouble t2;
    ADdouble t3;
    ADdouble t4;

    t2 = CppAD::cos(theta);
    t3 = CppAD::sin(theta);
    t4 = L0 * L0;
    ADdouble result = ADdouble(1.0) /
        ((t2 * t2 * t4 * ADdouble(1.961363726333745E+42) + t3 * t3 * t4 * ADdouble(1.3550195472288669E+43)) + ADdouble(4.7915666704630959E+38)) *
        (((((T * ADdouble(1.811039369903676E+24) - Tp * ADdouble(1.811039369903676E+24)) - L0 * t3 * ADdouble(9.1225675140787972E+24)) +
                L0 * T * t2 * ADdouble(4.62970041693687E+25)) + t2 * t3 * t4 * (theta1 * theta1) * ADdouble(7.8528978472083187E+23)) *
            ADdouble(-1.475739525896764E+19));

    return result;
}

ADdouble x_ddot_ad(ADdouble L0, ADdouble T, ADdouble Tp, ADdouble theta, ADdouble theta1)
{
    ADdouble t2;
    ADdouble t3;
    ADdouble t4;
    ADdouble t5;
    ADdouble t6;
    ADdouble t7;
    ADdouble t8;
    ADdouble x2_tmp;

    /*     This function was generated by the Symbolic Math Toolbox version 9.3.
     */
    /*     2025-11-20 19:35:13 */
    t2 = CppAD::cos(theta);
    t3 = CppAD::sin(theta);
    t4 = L0 * L0;
    t5 = CppAD::pow(L0, ADdouble(3.0));
    t6 = theta1 * theta1;
    t7 = t2 * t2;
    t8 = t3 * t3;
    x2_tmp = T * t4;
    ADdouble result = ((((((T * ADdouble(1.653596944829927E+24) + CppAD::pow(t3, ADdouble(3.0)) * t5 * t6 * ADdouble(7.9318546099989269E+26)) +
                    L0 * T * t2 * ADdouble(1.5644683648913071E+27)) - L0 * Tp * t2 * ADdouble(1.5644683648913071E+27)) +
                L0 * t3 * t6 * ADdouble(2.8048311378205221E+22)) + x2_tmp * t7 * ADdouble(4.6762496226853713E+28)) +
            ((x2_tmp * t8 * ADdouble(4.6762496226853713E+28) - t2 * t3 * t4 * ADdouble(7.880540047630492E+27)) +
                t3 * t5 * t6 * t7 * ADdouble(7.9318546099989269E+26))) * ADdouble(1.441151880758559E+16) /
        ((t4 * t7 * ADdouble(1.961363726333745E+42) + t4 * t8 * ADdouble(1.3550195472288669E+43)) + ADdouble(4.7915666704630959E+38));

    return result;
}

ADdouble phi_ddot_ad(ADdouble Tp)
{
    ADdouble result = Tp * ADdouble(2953.8432454466511);
    return result;
}
