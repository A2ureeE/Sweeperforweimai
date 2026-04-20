#include "Dynamics.h"
#include <cmath>

// [5状态模型] 连续时间常微分方程
// x: [x, y, v, yaw, steer]
// u: [acc, steer_rate]
Vector5d Dynamics::continuous_bicycle_ode(const Vector5d& x, const Eigen::Vector2d& u, double wheelbase) {
    double v = x(2);
    double yaw = x(3);
    double steer = x(4); // [新增] 转向角是状态
    
    double acc = u(0);
    double steer_rate = u(1); // [新增] 控制量是转向速率

    Vector5d x_dot;
    x_dot(0) = v * cos(yaw);
    x_dot(1) = v * sin(yaw);
    x_dot(2) = acc;
    // 运动学模型：yaw_rate = v * tan(delta) / L
    x_dot(3) = v * tan(steer) / wheelbase;
    // 转向角的变化率直接等于控制量
    x_dot(4) = steer_rate;
    
    return x_dot;
}

// RK4 离散化 (适配 5 维向量)
Vector5d Dynamics::bicycle_model(const Vector5d& cur_x, const Eigen::Vector2d& cur_u, double dt, double wheelbase) {
    Vector5d k1 = continuous_bicycle_ode(cur_x, cur_u, wheelbase);
    Vector5d k2 = continuous_bicycle_ode(cur_x + 0.5 * dt * k1, cur_u, wheelbase);
    Vector5d k3 = continuous_bicycle_ode(cur_x + 0.5 * dt * k2, cur_u, wheelbase);
    Vector5d k4 = continuous_bicycle_ode(cur_x + dt * k3, cur_u, wheelbase);
    
    return cur_x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// [核心] 计算雅可比矩阵 A (5x5) 和 B (5x2)
void Dynamics::get_N_steps_bicycle_model_derivatives(
    const std::vector<Vector5d>& x_traj,
    const std::vector<Eigen::Vector2d>& u_traj,
    double dt, double wheelbase, int N,
    std::vector<Matrix5d>& df_dx,
    std::vector<Matrix52d>& df_du)
{
    // [优化] 预分配内存，避免 vector 扩容开销
    if (df_dx.size() != (size_t)N) df_dx.resize(N);
    if (df_du.size() != (size_t)N) df_du.resize(N);

    for (int i = 0; i < N; ++i) {
        double v = x_traj[i](2);
        double yaw = x_traj[i](3);
        double steer = x_traj[i](4); // 当前转向角状态
        
        // 预计算三角函数
        double cos_yaw = cos(yaw);
        double sin_yaw = sin(yaw);
        double tan_steer = tan(steer);
        // d(tan(x))/dx = 1 + tan^2(x) = 1/cos^2(x)
        double sec_sq_steer = 1.0 + tan_steer * tan_steer;

        // --- A 矩阵 (df/dx) 5x5 ---
        // 状态顺序: x, y, v, yaw, steer
        df_dx[i] = Matrix5d::Identity();
        
        // 1. d(x_next) / d(...)
        // x_next ≈ x + (v*cos(yaw))*dt
        df_dx[i](0, 2) = cos_yaw * dt;        // d(x)/d(v)
        df_dx[i](0, 3) = -v * sin_yaw * dt;   // d(x)/d(yaw)

        // 2. d(y_next) / d(...)
        // y_next ≈ y + (v*sin(yaw))*dt
        df_dx[i](1, 2) = sin_yaw * dt;        // d(y)/d(v)
        df_dx[i](1, 3) = v * cos_yaw * dt;    // d(y)/d(yaw)

        // 3. d(v_next) / d(...) 
        // v_next = v + a*dt -> 这一行只有对角线是1，前面已设 Identity

        // 4. d(yaw_next) / d(...)
        // yaw_next ≈ yaw + (v * tan(steer) / L) * dt
        df_dx[i](3, 2) = tan_steer / wheelbase * dt;        // d(yaw)/d(v)
        df_dx[i](3, 4) = v * sec_sq_steer / wheelbase * dt; // d(yaw)/d(steer) [关键项]

        // 5. d(steer_next) / d(...)
        // steer_next = steer + steer_rate*dt -> 这一行也只有对角线是1

        // --- B 矩阵 (df/du) 5x2 ---
        // 控制顺序: acc, steer_rate
        df_du[i] = Matrix52d::Zero();
        
        // 1. d(v_next) / d(acc)
        df_du[i](2, 0) = dt; 

        // 2. d(steer_next) / d(steer_rate)
        df_du[i](4, 1) = dt;
    }
}