/**
@file           : NMPC_casADi.h
@author         : Chen Haoran
*/
#ifndef WHEEL_LEG_SYS_NMPC_CASADI_H
#define WHEEL_LEG_SYS_NMPC_CASADI_H

/* Includes ------------------------------------------------------------------*/
#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <cstring> // for memcpy

/* Class ---------------------------------------------------------------------*/
class class_NMPC_casADi
{
public:
    class_NMPC_casADi();

    [[nodiscard]] std::tuple<std::vector<double>, casadi::DM, casadi::DM> solve(
        const std::vector<double>& x_init,
        const std::vector<std::vector<double>>& reference_trajectory,
        double L0, double L1) const;

    [[nodiscard]] std::tuple<std::vector<double>, casadi::DM, casadi::DM> solve1(
        const std::vector<double>& x_init,
        const std::vector<std::vector<double>>& reference_trajectory,
        double L0, double L1, const std::vector<double>& guess = {}) const;

private:
    int n = 0;
    int p = 0;
    int N = 0;
    double dt = 0;

    // 边界条件
    std::vector<double> x_min;
    std::vector<double> x_max;
    std::vector<double> u_min;
    std::vector<double> u_max;

    // CasADi 函数对象 (通用，不需要改类型)
    casadi::Function dynamics;
    casadi::Function solver;

    // 内部构建函数
    void build_dynamics();
    [[nodiscard]] casadi::Function getDiscreteDynamics() const;
    void build_optimizer();
};

#endif //WHEEL_LEG_SYS_NMPC_CASADI_H