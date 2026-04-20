#pragma once
#include <Eigen/Dense>
#include "Utils.h"

class Constraints {
public:
    static double exp_barrier(double c, double q1, double q2);
    
    static void exp_barrier_derivative_and_Hessian(
        double c, const Eigen::MatrixXd& c_dot, double q1, double q2,
        Eigen::MatrixXd& b_dot, Eigen::MatrixXd& b_ddot);
        
    static double get_bound_constr(double var, double bound, const std::string& type);
    
    // --- 道路边界约束 ---
    // [修改] 输入 x 变为 Vector5d
    static double get_road_corridor_constr(const Vector5d& x, const SplineProps& proj, double road_width, double safe_margin);
    
    // [修改] 输出导数 c_dot 变为 1x5 矩阵 (Eigen::MatrixXd 兼容性更好)
    static void get_road_corridor_constr_and_deriv(const Vector5d& x, const SplineProps& proj, double road_width, double safe_margin, double& c, Eigen::MatrixXd& c_dot);

    // --- 障碍物约束 ---
    // [修改] 输入 x 变为 Vector5d
    static void get_obstacle_avoidance_constr(
        const Vector5d& ego_state, 
        const Eigen::Vector4d& obs_state, 
        double ego_wheelbase, double ego_width, 
        double obs_width, double obs_length, double d_safe,
        double& front_margin, double& rear_margin);

    // [修改] 输出导数变为 Eigen::MatrixXd (实际 1x5)
    static void get_obstacle_avoidance_constr_derivatives(
        const Vector5d& ego_state, 
        const Eigen::Vector4d& obs_state, 
        double ego_wheelbase, double ego_width, 
        double obs_width, double obs_length, double d_safe,
        Eigen::MatrixXd& front_deriv, 
        Eigen::MatrixXd& rear_deriv);
};