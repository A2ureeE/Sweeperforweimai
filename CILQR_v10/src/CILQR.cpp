#include "CILQR.h"
#include "Constraints.h"
#include "Dynamics.h"
#include <iostream>
#include <algorithm>
#include <cmath>
/**
 * @brief CILQR 构造函数
 * 初始化权重矩阵 Q (状态代价) 和 R (控制代价)
 */
CILQR::CILQR(const Config& config) : cfg(config) {
    // --- 初始化状态权重矩阵 Q (5x5) ---
    // 对角线元素分别对应: [x, y, v, yaw, steer]
    state_weight.setZero();
    state_weight.diagonal() << cfg.mpc.w_pos, cfg.mpc.w_pos, cfg.mpc.w_vel, cfg.mpc.w_yaw_process, cfg.mpc.w_steer;  
    // --- 初始化终端状态权重矩阵 Q_term (5x5) ---
    // 用于惩罚预测时域末端的偏差，确保能够收敛到目标   
    terminal_state_weight.setZero();
    terminal_state_weight.diagonal() << cfg.mpc.w_pos_terminal, cfg.mpc.w_pos_terminal, cfg.mpc.w_vel_terminal, cfg.mpc.w_yaw_terminal, cfg.mpc.w_steer_terminal;
    // --- 初始化控制权重矩阵 R (2x2) ---
    // 对角线元素分别对应: [acc, steer_rate]
    // 较大的权重会抑制控制量的剧烈变化，使控制更平滑    
    ctrl_weight.setZero();
    ctrl_weight.diagonal() << cfg.mpc.w_acc, cfg.mpc.w_steer_rate;
    
    last_start_idx = 0;
}

/**
 * @brief 获取特定时刻 t 的参考状态
 * * 结合 DP 生成的参考轨迹 (Reference Trajectory) 和样条曲线 (Spline) 属性，
 * 计算 t 时刻期望的 [x, y, v, yaw, steer]。
 * * @param t 预测时间
 * @param ref_traj DP 规划出的粗略轨迹点集
 * @param proj 当前预测点在样条曲线上的投影属性 (用于计算参考转向角)
 * @return Vector5d 参考状态向量
 */
Vector5d CILQR::get_ref_state_at_t(double t, const std::vector<TrajectoryPoint>& ref_traj, const SplineProps& proj) {
    // 默认值：如果找不到对应时间的参考点，则使用样条曲线上的几何信息
    double ref_v = cfg.road.velo_ref; 
    double ref_x = proj.pos.x();
    double ref_y = proj.pos.y();
    double ref_yaw = atan2(proj.tangent.y(), proj.tangent.x());
    double ref_kappa = proj.curvature; 

    // 如果有 DP 轨迹，则在时间域上进行插值
    if (!ref_traj.empty()) {
        // 二分查找找到 t 时刻前后的轨迹点
        auto it = std::lower_bound(ref_traj.begin(), ref_traj.end(), t, 
            [](const TrajectoryPoint& p, double val) { return p.t < val; });
        
        if (it == ref_traj.begin()) {
            ref_v = it->v; ref_x = it->x; ref_y = it->y; ref_yaw = it->theta; ref_kappa = it->kappa;
        } else if (it == ref_traj.end()) {
            // 超出轨迹末端，沿切线外推
            const auto& last_p = ref_traj.back();
            double dt = t - last_p.t;
            ref_v = last_p.v; ref_yaw = last_p.theta; ref_kappa = last_p.kappa;
            ref_x = last_p.x + ref_v * std::cos(ref_yaw) * dt;
            ref_y = last_p.y + ref_v * std::sin(ref_yaw) * dt;
        } else {
            // 线性插值
            auto prev = it - 1;
            double dt = it->t - prev->t;
            if (dt < 1e-5) dt = 1e-5;
            double ratio = (t - prev->t) / dt;
            ref_v = prev->v + ratio * (it->v - prev->v);
            ref_x = prev->x + ratio * (it->x - prev->x);
            ref_y = prev->y + ratio * (it->y - prev->y);
            double d_theta = angdiff(it->theta, prev->theta);// 处理角度跳变 (-pi 到 pi)
            ref_yaw = prev->theta + ratio * d_theta;
            ref_kappa = prev->kappa + ratio * (it->kappa - prev->kappa);
        }
    }

    // 根据运动学自行车模型公式: tan(delta) = L * kappa
    // 反算参考转向角，引导车辆顺应道路弯度，这是平滑过弯的关键。
    double ref_steer = std::atan(cfg.vehicle.wheelbase * ref_kappa);
    
    Vector5d ref;
    ref << ref_x, ref_y, ref_v, ref_yaw, ref_steer;
    return ref;
}

/**
 * @brief CILQR 求解器主入口
 * * 执行 Constrained Iterative LQR 算法。
 * 流程:
 * 1. 初始化/热启动 (Warm Start)
 * 2. 迭代循环 (Backward Pass -> Forward Pass)
 * 3. 动态调整正则化参数 lambda
 */
void CILQR::solve(const Vector5d& x0, Utils& utils, PathData& path_data, 
           const std::vector<TrajectoryPoint>& ref_traj, 
           const std::vector<Obstacle>& obs_list,
           const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
           const std::vector<Vector5d>& prev_opti_x,
           const std::vector<Eigen::Vector2d>& prev_opti_u,
           std::vector<Eigen::Vector2d>& opti_u,
           std::vector<Vector5d>& opti_x)
{
    // 0. 安全检查：如果初始状态无效，直接返回
    if (std::isnan(x0.sum()) || std::isinf(x0.sum())) {
        opti_u.assign(cfg.mpc.N, Eigen::Vector2d::Zero());
        opti_x.assign(cfg.mpc.N + 1, x0);
        return;
    }
    
    last_iter_s_values.clear();

    // 1. 初始化控制序列 (Warm Start)
    // 利用上一帧的解向前平移一步，作为当前帧的初值，显著加速收敛
    std::vector<Eigen::Vector2d> u = get_nominal_control(prev_opti_u);

    // 2. 初始前向模拟 (Rollout)
    // 根据当前控制 u 积分得到状态轨迹 x
    std::vector<Vector5d> x = rollout_trajectory(x0, u);

    // 用于一致性代价 (Consistency Cost) 的参考，防止控制量突变
    std::vector<Vector5d> consistency_ref_x = prev_opti_x;
    if (consistency_ref_x.empty()) consistency_ref_x = x;
    
    // 3. 计算所有点的投影 (用于计算横向误差、道路边界约束)
    std::vector<SplineProps> projections = calculate_all_projections(x, path_data);
    // 4. 计算初始总代价 Cost
    double J = get_total_cost(u, x, projections, ref_traj, obs_list, obs_pred_list, consistency_ref_x);
    // 初始化正则化参数 lambda
    double lamb = cfg.iteration.init_lamb;

    // --- iLQR 主循环 ---
    for (int itr = 0; itr < cfg.iteration.max_iter; ++itr) {
        std::vector<Eigen::Vector2d> k_gain(cfg.mpc.N);// 前馈增益
        std::vector<Matrix25d> K_gain(cfg.mpc.N); // 反馈增益 K (2x5)
        
        // 每次迭代前重新计算投影，因为 x 变了，参考点也会微调
        projections = calculate_all_projections(x, path_data);
        
        // --- Backward Pass (反向传播) ---
        // 利用 Riccati 方程计算控制律 u = k + K*dx
        // 同时计算 Cost function 的一阶和二阶导数
        bool bp_ok = backward_pass(u, x, lamb, projections, ref_traj, obs_list, obs_pred_list, consistency_ref_x, k_gain, K_gain);
         
        // 如果 Backward Pass 失败 (通常是因为 Hessian 非正定)，增大正则化参数 lambda 重新尝试       
        if (!bp_ok) {
            lamb *= cfg.iteration.lamb_amplify;
            if (lamb > cfg.iteration.max_lamb) break;
            continue;
        }

        // --- Forward Pass (前向传播 / 线搜索) ---
        bool fp_ok = false;
        std::vector<Eigen::Vector2d> new_u;
        std::vector<Vector5d> new_x;
        double new_J = 0;
        
        // 尝试不同的步长 alpha (Line Search)，保证 Cost 单调下降
        for (double alpha : cfg.iteration.alpha_options) {
            forward_pass(u, x, k_gain, K_gain, alpha, new_u, new_x);
            
            // 检查数值有效性
            bool has_nan = false;
            for(const auto& s : new_x) if(std::isnan(s.sum())) has_nan = true;
            if(has_nan) continue; 

            // 计算新轨迹的代价
            std::vector<SplineProps> new_projections = calculate_all_projections(new_x, path_data);
            double tJ = get_total_cost(new_u, new_x, new_projections, ref_traj, obs_list, obs_pred_list, consistency_ref_x);
            
            // 如果代价降低，接受该更新
            if (tJ < J) {
                u = new_u;
                x = new_x;
                J = tJ;
                fp_ok = true;
                lamb *= cfg.iteration.lamb_decay;
                break;
            }
        }
        
        // 如果所有 alpha 都无法降低代价
        if (!fp_ok) {
            lamb *= cfg.iteration.lamb_amplify;
            if (lamb > cfg.iteration.max_lamb) break;
        } else {// 收敛判定：代价变化极小则提前退出
            if (std::abs(J - new_J) < cfg.iteration.tol) break;
        }

        // 相对收敛判定
        if (std::abs(J - new_J) < cfg.iteration.tol || std::abs(J - new_J) / (J + 1e-5) < 1e-4) {
            break;
        }
    }
    opti_u = u;
    opti_x = x;
}

/**
 * @brief 计算轨迹上所有点在参考样条线上的投影
 * * 包含“热启动”优化：利用上一帧计算的 s 值作为初值，加速 Newton 迭代。
 */
std::vector<SplineProps> CILQR::calculate_all_projections(const std::vector<Vector5d>& x, PathData& path_data) {
    int N = x.size();
    std::vector<SplineProps> projections(N);
    std::vector<double> current_s_values(N);

    int total_wp = path_data.waypoints.size();
    if (total_wp == 0) return projections;

    int current_search_idx = last_start_idx; 
    int search_window = 50; 
    // 检查是否有上一帧的 s 值缓存
    bool use_warm_start = (last_iter_s_values.size() == N);

    for (int i = 0; i < N; ++i) {
        Eigen::Vector2d pos = x[i].head(2);
        if (std::isnan(pos.x()) || std::isnan(pos.y())) {
             projections[i] = Utils::get_spline_properties(path_data.spline, 0);
             continue;
        }

        double t_final = 0.0;
        if (use_warm_start) {
            // 优化：使用缓存的 s 进行局部牛顿迭代
            double t_guess = last_iter_s_values[i];
            t_final = Utils::project_to_spline_newton(pos, path_data.spline, t_guess);
        } else {
            // 冷启动：暴力搜索最近点 + 牛顿迭代
            double min_dist = 1e9;
            int best_idx = current_search_idx;
            for (int j = -5; j < search_window; j++) {
                int curr_idx = (current_search_idx + j) % total_wp;
                if (curr_idx < 0) curr_idx += total_wp;
                double d = (path_data.waypoints[curr_idx] - pos).norm();
                if (d < min_dist) { min_dist = d; best_idx = curr_idx; }
            }
            t_final = Utils::project_to_spline_newton(pos, path_data.spline, path_data.t_params[best_idx]);
            current_search_idx = best_idx;
            if (i == 0) last_start_idx = best_idx;
        }
        projections[i] = Utils::get_spline_properties(path_data.spline, t_final);
        current_s_values[i] = t_final; 
    }
    // 更新缓存
    last_iter_s_values = current_s_values;
    return projections;
}

// 获取初始控制序列（移位操作）
std::vector<Eigen::Vector2d> CILQR::get_nominal_control(const std::vector<Eigen::Vector2d>& prev_u) {
    std::vector<Eigen::Vector2d> u(cfg.mpc.N, Eigen::Vector2d::Zero());
    if (!prev_u.empty() && prev_u.size() == (size_t)cfg.mpc.N) {
        // 将 u[k+1] 移到 u[k]，并在末尾复制最后一个控制量
        for (int i = 0; i < cfg.mpc.N - 1; ++i) u[i] = prev_u[i+1];
        u[cfg.mpc.N - 1] = prev_u.back();
    }
    return u;
}

// 正向动力学积分 (Rollout)
std::vector<Vector5d> CILQR::rollout_trajectory(const Vector5d& x0, const std::vector<Eigen::Vector2d>& u) {
    std::vector<Vector5d> x(cfg.mpc.N + 1);
    x[0] = x0;
    for (int i = 0; i < cfg.mpc.N; ++i) {
        x[i+1] = Dynamics::bicycle_model(x[i], u[i], cfg.mpc.dt, cfg.vehicle.wheelbase);
    }
    return x;
}

double CILQR::angdiff(double a, double b) {
    return atan2(sin(a - b), cos(a - b));
}

/**
 * @brief 计算总代价 (Cost Function)
 * * J = J_tracking + J_control + J_consistency + J_constraints(Barrier)
 */
double CILQR::get_total_cost(const std::vector<Eigen::Vector2d>& u, 
                      const std::vector<Vector5d>& x,
                      const std::vector<SplineProps>& projections, 
                      const std::vector<TrajectoryPoint>& ref_traj,
                      const std::vector<Obstacle>& obs_list,
                      const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
                      const std::vector<Vector5d>& prev_opti_x) 
{
    double total_cost = 0;

    for (int i = 0; i < cfg.mpc.N; ++i) {
        double t_predict = i * cfg.mpc.dt; 
        Vector5d ref_state = get_ref_state_at_t(t_predict, ref_traj, projections[i]);
        Vector5d dx = x[i] - ref_state;
        dx(3) = angdiff(x[i](3), ref_state(3));// 角度差分特殊处理
        // 1. 状态追踪代价 (x^T Q x)
        total_cost += dx.transpose() * state_weight * dx;
        // 2. 控制量代价 (u^T R u)
        total_cost += u[i].transpose() * ctrl_weight * u[i];
        // 3. 一致性代价 (减小与上一帧解的抖动)
        if (i < (int)prev_opti_x.size()) {
            Vector5d dev = x[i] - prev_opti_x[i];
            total_cost += cfg.mpc.w_consistency * dev.dot(dev);
        }
        
        // --- 4. 控制量软约束 (Barrier Functions) ---
        // u: [acc, steer_rate]
        double acc = u[i](0);
        double steer_rate = u[i](1);
        // 加速度限制
        total_cost += Constraints::exp_barrier(acc - cfg.vehicle.a_max, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        total_cost += Constraints::exp_barrier(cfg.vehicle.a_min - acc, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        // 转向速率限制 (Control Constraint)
        total_cost += Constraints::exp_barrier(steer_rate - cfg.vehicle.max_steer_rate, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        total_cost += Constraints::exp_barrier(-cfg.vehicle.max_steer_rate - steer_rate, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
    }

    // 5. 终端代价 (Terminal Cost)
    int N = cfg.mpc.N;
    double t_term = N * cfg.mpc.dt;
    Vector5d ref_term = get_ref_state_at_t(t_term, ref_traj, projections[N]);
    Vector5d dx_term = x[N] - ref_term;
    dx_term(3) = angdiff(x[N](3), ref_term(3));
    total_cost += dx_term.transpose() * terminal_state_weight * dx_term;
    
    // --- 6. 状态量软约束 (State Constraints) ---
    for (int k = 1; k <= N; ++k) {
        // 速度限制
        total_cost += Constraints::exp_barrier(x[k](2) - cfg.vehicle.velo_max, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        total_cost += Constraints::exp_barrier(cfg.vehicle.velo_min - x[k](2), cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        // 转向角限制 (State Constraint)
        double steer = x[k](4);
        total_cost += Constraints::exp_barrier(steer - cfg.vehicle.stl_lim, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        total_cost += Constraints::exp_barrier(-cfg.vehicle.stl_lim - steer, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        // 道路边界约束
        total_cost += Constraints::exp_barrier(Constraints::get_road_corridor_constr(x[k], projections[k], cfg.road.width, cfg.mpc.road_safe_margin), cfg.mpc.road_exp_q1, cfg.mpc.road_exp_q2);
        
        // 障碍物避让约束 (对每个障碍物的前后两点进行检测)
        for (size_t o = 0; o < obs_list.size(); ++o) {
            double m_front, m_rear;
            Constraints::get_obstacle_avoidance_constr(x[k], obs_pred_list[o][k], cfg.vehicle.wheelbase, cfg.vehicle.width, obs_list[o].width, obs_list[o].length, obs_list[o].d_safe, m_front, m_rear);
            total_cost += Constraints::exp_barrier(m_front, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
            total_cost += Constraints::exp_barrier(m_rear, cfg.mpc.exp_q1, cfg.mpc.exp_q2);
        }
    }
    return total_cost;
}

/**
 * @brief 计算总代价的一阶导 (Gradient) 和二阶导 (Hessian)
 * * 这是 iLQR 的核心部分。我们使用解析导数（Analytical Derivatives）而非数值差分，
 * 且通过链式法则将 Barrier Function 的导数叠加到 Cost 导数上。
 * * 输出:
 * l_x: dJ/dx, l_xx: d^2J/dx^2 (状态相关)
 * l_u: dJ/du, l_uu: d^2J/du^2 (控制相关)
 */
void CILQR::get_total_cost_derivatives(
    const std::vector<Eigen::Vector2d>& u, 
    const std::vector<Vector5d>& x,
    const std::vector<SplineProps>& projections,
    const std::vector<TrajectoryPoint>& ref_traj,
    const std::vector<Obstacle>& obs_list,
    const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
    const std::vector<Vector5d>& prev_opti_x,
    std::vector<Vector5d>& l_x,
    std::vector<Matrix5d>& l_xx,
    std::vector<Eigen::Vector2d>& l_u,
    std::vector<Eigen::Matrix2d>& l_uu)
{
    int N = cfg.mpc.N;
    l_x.assign(N + 1, Vector5d::Zero());
    l_xx.assign(N + 1, Matrix5d::Zero());
    l_u.assign(N, Eigen::Vector2d::Zero());
    l_uu.assign(N, Eigen::Matrix2d::Zero());

    // 1. 基础二次型代价的导数 (Tracking Cost)
    // J = (x-ref)^T Q (x-ref) -> dJ/dx = 2Q(x-ref), d2J/dx2 = 2Q
    for (int i = 0; i < N; ++i) {
        double t_predict = i * cfg.mpc.dt;
        Vector5d ref_state = get_ref_state_at_t(t_predict, ref_traj, projections[i]);
        Vector5d dx = x[i] - ref_state;
        dx(3) = angdiff(x[i](3), ref_state(3));
        
        l_x[i] += 2.0 * state_weight * dx;
        l_xx[i] += 2.0 * state_weight;
        l_u[i] += 2.0 * ctrl_weight * u[i];
        l_uu[i] += 2.0 * ctrl_weight;

        if (i < (int)prev_opti_x.size()) {
            l_x[i] += 2.0 * cfg.mpc.w_consistency * (x[i] - prev_opti_x[i]);
            l_xx[i] += 2.0 * cfg.mpc.w_consistency * Matrix5d::Identity();
        }
    }
    // 终端代价导数
    double t_term = N * cfg.mpc.dt;
    Vector5d ref_term = get_ref_state_at_t(t_term, ref_traj, projections[N]);
    Vector5d dx_term = x[N] - ref_term;
    dx_term(3) = angdiff(x[N](3), ref_term(3));
    l_x[N] += 2.0 * terminal_state_weight * dx_term;
    l_xx[N] += 2.0 * terminal_state_weight;
    
    // --- 2. 控制量约束的导数 (Barrier Gradients) ---
    for (int k = 0; k < N; ++k) {
        Eigen::MatrixXd b_dot, b_ddot;
        Eigen::MatrixXd c_dot_u = Eigen::MatrixXd::Zero(1, 2);
        // 加速度约束导数
        c_dot_u << 1, 0;// acc 对 u 的导数
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(u[k](0), cfg.vehicle.a_max, "upper"), c_dot_u, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_u[k] += b_dot.transpose(); l_uu[k] += b_ddot;
        c_dot_u << -1, 0;
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(u[k](0), cfg.vehicle.a_min, "lower"), c_dot_u, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_u[k] += b_dot.transpose(); l_uu[k] += b_ddot;
        
        // 转向速率约束导数
        c_dot_u << 0, 1;// steer_rate 对 u 的导数
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(u[k](1), cfg.vehicle.max_steer_rate, "upper"), c_dot_u, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_u[k] += b_dot.transpose(); l_uu[k] += b_ddot;
        c_dot_u << 0, -1;
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(u[k](1), -cfg.vehicle.max_steer_rate, "lower"), c_dot_u, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_u[k] += b_dot.transpose(); l_uu[k] += b_ddot;
    }
    
    // --- 3. 状态量约束的导数 ---
    for (int k = 1; k <= N; ++k) {
        Eigen::MatrixXd b_dot, b_ddot;
        Eigen::MatrixXd c_dot_v = Eigen::MatrixXd::Zero(1, 5);
        
        // 速度约束导数
        c_dot_v << 0, 0, 1, 0, 0; 
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(x[k](2), cfg.vehicle.velo_max, "upper"), c_dot_v, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_x[k] += b_dot.transpose(); l_xx[k] += b_ddot;
        c_dot_v << 0, 0, -1, 0, 0;
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(x[k](2), cfg.vehicle.velo_min, "lower"), c_dot_v, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_x[k] += b_dot.transpose(); l_xx[k] += b_ddot;
        
        // 转向角约束导数 (现在是 State Constraint)
        c_dot_v << 0, 0, 0, 0, 1;
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(x[k](4), cfg.vehicle.stl_lim, "upper"), c_dot_v, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_x[k] += b_dot.transpose(); l_xx[k] += b_ddot;
        c_dot_v << 0, 0, 0, 0, -1;
        Constraints::exp_barrier_derivative_and_Hessian(Constraints::get_bound_constr(x[k](4), -cfg.vehicle.stl_lim, "lower"), c_dot_v, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
        l_x[k] += b_dot.transpose(); l_xx[k] += b_ddot;

        // 道路边界约束导数
        double c_road;
        Eigen::MatrixXd c_dot_road;
        Constraints::get_road_corridor_constr_and_deriv(x[k], projections[k], cfg.road.width, cfg.mpc.road_safe_margin, c_road, c_dot_road);
        // 链式法则：Barrier对Cost的贡献
        Constraints::exp_barrier_derivative_and_Hessian(c_road, c_dot_road, cfg.mpc.road_exp_q1, cfg.mpc.road_exp_q2, b_dot, b_ddot);
        l_x[k] += b_dot.transpose(); l_xx[k] += b_ddot;

        // 障碍物避让约束导数
        for (size_t o = 0; o < obs_list.size(); ++o) {
            double m_front, m_rear;
            // 计算约束值
            Constraints::get_obstacle_avoidance_constr(x[k], obs_pred_list[o][k], cfg.vehicle.wheelbase, cfg.vehicle.width, obs_list[o].width, obs_list[o].length, obs_list[o].d_safe, m_front, m_rear);
            // 计算几何导数
            Eigen::MatrixXd front_deriv, rear_deriv;
            Constraints::get_obstacle_avoidance_constr_derivatives(x[k], obs_pred_list[o][k], cfg.vehicle.wheelbase, cfg.vehicle.width, obs_list[o].width, obs_list[o].length, obs_list[o].d_safe, front_deriv, rear_deriv);
            // 叠加 Barrier 导数
            Constraints::exp_barrier_derivative_and_Hessian(m_front, front_deriv, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
            l_x[k] += b_dot.transpose(); l_xx[k] += b_ddot;

            Constraints::exp_barrier_derivative_and_Hessian(m_rear, rear_deriv, cfg.mpc.exp_q1, cfg.mpc.exp_q2, b_dot, b_ddot);
            l_x[k] += b_dot.transpose(); l_xx[k] += b_ddot;
        }
    }
}

/**
 * @brief Backward Pass (Riccati 递归)
 * * 逆序计算 Value Function 的二次近似 (Q函数)，从而得到最优控制律的增益 k 和 K。
 * * @param lamb 正则化参数 (Levenberg-Marquardt)
 * @return true 如果所有 Quu 矩阵正定 (成功)，false 否则
 */
bool CILQR::backward_pass(const std::vector<Eigen::Vector2d>& u, 
                   const std::vector<Vector5d>& x,
                   double lamb,
                   const std::vector<SplineProps>& projections, 
                   const std::vector<TrajectoryPoint>& ref_traj, 
                   const std::vector<Obstacle>& obs_list,
                   const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
                   const std::vector<Vector5d>& prev_opti_x,
                   std::vector<Eigen::Vector2d>& k_gain,
                   std::vector<Matrix25d>& K_gain) // [修复] Matrix25d
{
    int N = cfg.mpc.N;

    // 1. 线性化动力学模型 (A, B 矩阵)
    std::vector<Matrix5d> df_dx;
    std::vector<Matrix52d> df_du;
    Dynamics::get_N_steps_bicycle_model_derivatives(x, u, cfg.mpc.dt, cfg.vehicle.wheelbase, N, df_dx, df_du);

    // 2. 二次化代价函数 (l_x, l_xx, l_u, l_uu)
    std::vector<Vector5d> l_x;
    std::vector<Matrix5d> l_xx;
    std::vector<Eigen::Vector2d> l_u;
    std::vector<Eigen::Matrix2d> l_uu;
    
    get_total_cost_derivatives(u, x, projections, ref_traj, obs_list, obs_pred_list, prev_opti_x, l_x, l_xx, l_u, l_uu);

    // 初始化终端 Value Function: V_N = l_N(x_N)
    Vector5d Vx = l_x[N];
    Matrix5d Vxx = l_xx[N];

    // 正则化项：lambda * I
    Matrix5d regI = lamb * Matrix5d::Identity();
    
    // 3. 反向递归求解
    for (int i = N - 1; i >= 0; --i) {
        // Q函数的梯度 Qx, Qu
        Vector5d Qx = l_x[i] + df_dx[i].transpose() * Vx;
        Eigen::Vector2d Qu = l_u[i] + df_du[i].transpose() * Vx;
        // Q函数的 Hessian Qxx, Quu, Qux
        Matrix5d Qxx = l_xx[i] + df_dx[i].transpose() * Vxx * df_dx[i];
        Eigen::Matrix2d Quu = l_uu[i] + df_du[i].transpose() * Vxx * df_du[i];
        Matrix25d Qux = df_du[i].transpose() * Vxx * df_dx[i]; // [修复] Matrix25d (2x5)
        
        // 正则化：增强 Quu 的对角占优，保证可逆且数值稳定
        // 相当于在控制代价中增加了阻尼
        Eigen::Matrix2d Quu_reg = Quu + df_du[i].transpose() * regI * df_du[i];
        
        // Cholesky 分解求解线性方程 Quu * k = -Qu
        Eigen::LLT<Eigen::Matrix2d> llt(Quu_reg);
        if (llt.info() == Eigen::NumericalIssue) {
            // 如果分解失败(非正定)，尝试增加微小量或返回失败以触发 lambda 增大
            Quu_reg += 1e-4 * Eigen::Matrix2d::Identity();
            llt.compute(Quu_reg);
            // 如果仍然失败，返回 false，主循环会增大 lambda
            if (llt.info() == Eigen::NumericalIssue) return false;
        }

        // 计算反馈增益 K 和前馈增益 k
        // u* = u_nom + k + K * (x - x_nom)
        Eigen::Vector2d k = -llt.solve(Qu);
        Matrix25d K = -llt.solve(Qux); // [修复] Matrix25d (2x5)
        
        k_gain[i] = k;
        K_gain[i] = K;

        // 更新 Value Function 供下一步使用
        // Vx = Qx + K^T Quu k + K^T Qu + Qux^T k
        Vx = Qx + K.transpose() * Quu * k + K.transpose() * Qu + Qux.transpose() * k;
        Vxx = Qxx + K.transpose() * Quu * K + K.transpose() * Qux + Qux.transpose() * K;
    }
    return true;
}

/**
 * @brief Forward Pass (前向传播)
 * * 使用计算出的增益 k 和 K，生成新的控制序列和状态轨迹。
 * u_new = u_nom + alpha * k + K * (x_new - x_nom)
 * * @param alpha 线搜索步长 (0 < alpha <= 1)
 */
void CILQR::forward_pass(const std::vector<Eigen::Vector2d>& u, 
                  const std::vector<Vector5d>& x,
                  const std::vector<Eigen::Vector2d>& k, 
                  const std::vector<Matrix25d>& K, 
                  double alpha, 
                  std::vector<Eigen::Vector2d>& new_u,
                  std::vector<Vector5d>& new_x) 
{
    new_u.resize(cfg.mpc.N);
    new_x.resize(cfg.mpc.N + 1);
    new_x[0] = x[0];

    for (int i = 0; i < cfg.mpc.N; ++i) {
        // 控制律更新
        Eigen::Vector2d u_curr = u[i] + alpha * k[i] + K[i] * (new_x[i] - x[i]);
        // 物理约束截断 (Hard Clip)
        // 虽然代价函数里有软约束，但为了确保仿真稳定性，这里做一次硬截断
        u_curr(0) = std::max(std::min(u_curr(0), cfg.vehicle.a_max), cfg.vehicle.a_min);
        u_curr(1) = std::max(std::min(u_curr(1), cfg.vehicle.max_steer_rate), -cfg.vehicle.max_steer_rate);
        
        new_u[i] = u_curr;
        // 动力学积分
        new_x[i+1] = Dynamics::bicycle_model(new_x[i], new_u[i], cfg.mpc.dt, cfg.vehicle.wheelbase);
    }
}