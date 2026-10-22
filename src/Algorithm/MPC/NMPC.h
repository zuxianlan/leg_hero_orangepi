/**
  ******************************************************************************
  * @file           : NMPC.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/20
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_NMPC_H
#define WHEEL_LEG_SYS_NMPC_H
/* Includes ------------------------------------------------------------------*/
#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <iostream>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

// CppAD definition of related types
typedef CppAD::AD<double> ADdouble;
typedef Eigen::Matrix<ADdouble, Eigen::Dynamic, 1> ADVector;
typedef Eigen::Matrix<ADdouble, Eigen::Dynamic, Eigen::Dynamic> ADMatrix;

// SQP result
typedef struct
{
    Eigen::VectorXd trajectory; // optimize trajectory [x0, u0, x1, u1, ..., xN]
    Eigen::VectorXd lambda_eq;
    Eigen::VectorXd lambda_ineq;
    double cost;
    int iterations;
    bool converged;
} SQPSolution;

// linearization at the current trajectory
struct LinearizationResult
{
    std::vector<Eigen::MatrixXd> A_k; // A_k = partial(f)/partial(x)
    std::vector<Eigen::MatrixXd> B_k; // B_k = partial(f)/partial(u)
    std::vector<Eigen::VectorXd> f_k; // f(x_k, u_k)
};

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/


class class_NMPC
{
public:
    class_NMPC(int n, int p, int y, int N, float dt);

    SQPSolution solve();

    void Set_x(const Eigen::VectorXd& x_);

    void Set_x_ref(const Eigen::VectorXd& x_ref_);

    void Set_u_ref(const Eigen::VectorXd& u_ref_);

    void Set_x_min(const Eigen::VectorXd& x_min_);

    void Set_x_max(const Eigen::VectorXd& x_max_);

    void Set_u_min(const Eigen::VectorXd& u_min_);

    void Set_u_max(const Eigen::VectorXd& u_max_);

    void Set_Q(const Eigen::MatrixXd& Q_);

    void Set_R(const Eigen::MatrixXd& R_);

    void Set_F(const Eigen::MatrixXd& F_);

    inline void Set_L(double L0, double L1);


private:
    // problem parameter
    int n = 0; // state dim
    int p = 0; // input dim
    int y = 0; // output dim
    int N = 0; // control horizon
    int c = 5;
    int nV = 0;
    int nC = 0;
    float dt = 0;

    int n_vars_ = 0;
    int n_eq_constraints_ = 0;
    int n_ineq_constraints_ = 0;

    int max_sqp_iterations_ = 10; // max qp cycle times
    double tolerance_ = 1e-4; // error tolerance
    double line_search_tol_ = 1e-4;
    double L[2] = {}; // left leg and right leg length

    SQPSolution solution = {};

    Eigen::VectorXd x;
    Eigen::VectorXd x_ref;
    Eigen::VectorXd u_ref;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd F;

    Eigen::VectorXd u_min;
    Eigen::VectorXd u_max;

    Eigen::VectorXd x_min;
    Eigen::VectorXd x_max;

    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    Eigen::MatrixXd A_mat;
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
    Eigen::VectorXd lba;
    Eigen::VectorXd uba;

    Eigen::MatrixXd A_eq;
    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_eq;
    Eigen::VectorXd b_ineq;

    [[nodiscard]] Eigen::VectorXd initialize_trajectory() const;

    [[nodiscard]] LinearizationResult linearize_at_trajectory(const Eigen::VectorXd& trajectory) const;

    void linearize_with_cppad(const Eigen::VectorXd& x, const Eigen::VectorXd& u, Eigen::MatrixXd& A, Eigen::MatrixXd& B) const;

    [[nodiscard]] Eigen::VectorXd build_dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;

    [[nodiscard]] ADVector build_dynamics_ad(const ADVector& x_ad, const ADVector& u_ad) const;

    void build_QP_subproblem(const LinearizationResult& lin, const Eigen::VectorXd& current_traj);

    void build_gauss_newton_hessian();

    Eigen::VectorXd build_gradient(const Eigen::VectorXd& current_traj);

    void build_equality_constraints(const LinearizationResult& lin, const Eigen::VectorXd& current_traj);

    void build_inequality_constraints();

    void build_variable_bounds();

    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> solve_QP_with_qpOASES();

    [[nodiscard]] double line_search(const Eigen::VectorXd& current_traj, const Eigen::VectorXd& direction) const;

    Eigen::MatrixXd update_hessian_BFGS(const Eigen::VectorXd& z_old, const Eigen::VectorXd& z_new,
                                        const Eigen::VectorXd& lambda_eq, const Eigen::VectorXd& lambda_ineq);

    [[nodiscard]] bool check_convergence(const Eigen::VectorXd& z_old, const Eigen::VectorXd& z_new,
                                         const Eigen::VectorXd& step) const;


    [[nodiscard]] Eigen::VectorXd get_state_from_trajectory(const Eigen::VectorXd& trajectory, int k) const;

    [[nodiscard]] Eigen::VectorXd get_control_from_trajectory(const Eigen::VectorXd& trajectory, int k) const;

    [[nodiscard]] Eigen::VectorXd get_x_ref_from_trajectory(const Eigen::VectorXd& trajectory, int k) const;

    [[nodiscard]] Eigen::VectorXd get_u_ref_from_trajectory(const Eigen::VectorXd& trajectory, int k) const;

    [[nodiscard]] double merit_function(const Eigen::VectorXd& trajectory) const;

    [[nodiscard]] double merit_function_directional_derivative(const Eigen::VectorXd& trajectory,
                                                               const Eigen::VectorXd& direction) const;

    [[nodiscard]] double compute_total_cost(const Eigen::VectorXd& trajectory) const;

    Eigen::VectorXd gradient_lagrangian(const Eigen::VectorXd& z, const Eigen::VectorXd& lambda_eq,
                                        const Eigen::VectorXd& lambda_ineq);

    Eigen::VectorXd compute_equilibrium_control(const Eigen::VectorXd& state) const;

    Eigen::VectorXd simplified_dynamics(const Eigen::VectorXd& state, const Eigen::VectorXd& control) const;
};


inline void class_NMPC::Set_L(const double L0, const double L1)
{
    L[0] = L0;
    L[1] = L1;
}

inline void class_NMPC::Set_x(const Eigen::VectorXd& x_)
{
    x = x_;
}

inline void class_NMPC::Set_x_ref(const Eigen::VectorXd& x_ref_)
{
    x_ref = x_ref_;
}

inline void class_NMPC::Set_u_ref(const Eigen::VectorXd& u_ref_)
{
    u_ref = u_ref_;
}

inline void class_NMPC::Set_x_min(const Eigen::VectorXd& x_min_)
{
    x_min = x_min_;
}

inline void class_NMPC::Set_x_max(const Eigen::VectorXd& x_max_)
{
    x_max = x_max_;
}

inline void class_NMPC::Set_u_min(const Eigen::VectorXd& u_min_)
{
    u_min = u_min_;
}

inline void class_NMPC::Set_u_max(const Eigen::VectorXd& u_max_)
{
    u_max = u_max_;
}

inline void class_NMPC::Set_Q(const Eigen::MatrixXd& Q_)
{
    Q = Q_;
}

inline void class_NMPC::Set_R(const Eigen::MatrixXd& R_)
{
    R = R_;
}

inline void class_NMPC::Set_F(const Eigen::MatrixXd& F_)
{
    F = F_;
}






#endif //WHEEL_LEG_SYS_NMPC_H
