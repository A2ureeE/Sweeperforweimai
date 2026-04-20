#pragma once
#include <Eigen/Dense>
#include <vector>
#include "Utils.h" // 包含 Vector5d 定义

class Dynamics {
public:
    // [修改] 连续时间自行车模型 (5状态: x, y, v, yaw, steer)
    static Vector5d continuous_bicycle_ode(const Vector5d& x, const Eigen::Vector2d& u, double wheelbase);
    
    // [修改] RK4 离散化 (返回 5状态)
    static Vector5d bicycle_model(const Vector5d& cur_x, const Eigen::Vector2d& cur_u, double dt, double wheelbase);
    
    // [修改] 计算 N 步线性化矩阵 (5x5 State Matrix, 5x2 Control Matrix)
    static void get_N_steps_bicycle_model_derivatives(
        const std::vector<Vector5d>& x_traj,
        const std::vector<Eigen::Vector2d>& u_traj,
        double dt, double wheelbase, int N,
        std::vector<Matrix5d>& df_dx,
        std::vector<Matrix52d>& df_du);
};