#include "DPPlanner.h"
#include <iostream>
#include <limits>
#include <cmath>
#include <algorithm>

DPPlanner::DPPlanner(const Config& cfg, StateTransformer& st) 
    : cfg_(cfg), st_(st) {}

std::vector<Eigen::Vector2d> DPPlanner::GetDPPathPoints() const {
    std::vector<Eigen::Vector2d> points;
    if (dp_path_sl_.empty()) return points;
    
    for (const auto& sl : dp_path_sl_) {
        FrenetState fs;
        fs.s = sl.x(); fs.l = sl.y();
        fs.l_prime = 0; fs.s_dot = 0; 
        fs.s_ddot = 0; fs.l_ddot = 0; fs.l_double_prime = 0; 
        CartesianState cs = st_.FrenetToCartesian(fs);
        points.push_back(Eigen::Vector2d(cs.x, cs.y));
    }
    return points;
}

// --- 核心规划函数 ---
std::vector<Eigen::Vector4d> DPPlanner::Plan(
    const Eigen::Vector4d& start_state_xy, 
    const std::vector<Obstacle>& obs_list,
    const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list, 
    double road_left_bound, 
    double road_right_bound
) 
{
    // 1. 将起点转换为 Frenet 坐标
    CartesianState cs_start;
    cs_start.x = start_state_xy(0); cs_start.y = start_state_xy(1);
    cs_start.v = start_state_xy(2); cs_start.theta = start_state_xy(3);
    cs_start.kappa = 0; cs_start.a = 0; 

    FrenetState fs_start = st_.CartesianToFrenet(cs_start);

    int rows = cfg_.dp.rows; 
    int cols = cfg_.dp.cols; 
    
    // --- 动态调整 DP 采样步长 ---
    double mpc_horizon_time = cfg_.mpc.N * cfg_.mpc.dt; 
    // 使用较大的参考速度来计算需要的距离，防止视界不足
    double ref_vel_for_horizon = std::max(cs_start.v, 15.0);
    double required_s_horizon = ref_vel_for_horizon * mpc_horizon_time * 1.5;
    double min_config_horizon = cols * cfg_.dp.sample_s_step;
    double target_total_s = std::max(required_s_horizon, min_config_horizon);
    
    double ds = target_total_s / cols;
    if (ds > 15.0) ds = 15.0; // 限制最大步长，防止拟合失真

    // --- [Fix 1: 保守的时间估算] ---
    // 使用 *较快* 的速度来估算到达时间。
    // 这意味着 DP 会查询 "较近未来" 的障碍物位置（此时障碍物可能还没离开）。
    // 这是一种保守策略，防止因低估车速而导致误判 "障碍物已离开"。
    double plan_v = std::max(cs_start.v, cfg_.road.velo_ref); 
    plan_v = std::max(plan_v, 10.0); // 最小规划速度

    std::vector<double> layer_times(cols + 1, 0.0);
    for (int j = 0; j <= cols; ++j) {
        layer_times[j] = (j * ds) / plan_v;
    }
    
    // --- 边界收缩逻辑 ---
    double vehicle_half_width = cfg_.vehicle.width / 2.0;
    double safe_margin = 0.8; 
    double boundary_buffer = vehicle_half_width + safe_margin; 
    double valid_min_l = road_right_bound + boundary_buffer;
    double valid_max_l = road_left_bound - boundary_buffer;
    
    if (valid_max_l < valid_min_l) {
        std::cerr << "[DPPlanner Warning] Road too narrow! Using center." << std::endl;
        valid_max_l = (road_left_bound + road_right_bound) / 2.0;
        valid_min_l = valid_max_l;
    }
    double total_sample_width = valid_max_l - valid_min_l;
    
    auto get_l_from_row_idx = [&](int idx) {
        if (rows <= 1) return (valid_min_l + valid_max_l) / 2.0;
        double ratio = (double)idx / (double)(rows - 1);
        return valid_min_l + ratio * total_sample_width;
    };

    // 2. 初始化 DP 表
    std::vector<std::vector<double>> dp_cost(cols + 1, std::vector<double>(rows, std::numeric_limits<double>::max()));
    std::vector<std::vector<int>> parent_index(cols + 1, std::vector<int>(rows, -1));

    // --- 第一层 ---
    // [OpenMP] 这里不需要 collapse，单层并行即可
    #pragma omp parallel for
    for (int i = 0; i < rows; ++i) {
        double l_sample = get_l_from_row_idx(i);
        double s_sample = fs_start.s + ds;
        Eigen::Vector3d start_state(fs_start.l, fs_start.l_prime, fs_start.l_double_prime);
        Eigen::Vector3d end_state(l_sample, 0.0, 0.0); 

        double cost = CalculateLinkCost(
            start_state, end_state, 
            fs_start.s, s_sample, 
            layer_times[0], layer_times[1], 
            obs_list, obs_pred_list, 
            valid_max_l, valid_min_l
        );
        dp_cost[1][i] = cost;
        parent_index[1][i] = -1;
    }

    // --- 后续层 ---
    for (int j = 2; j <= cols; ++j) {
        double s_curr = fs_start.s + j * ds;
        double s_prev = fs_start.s + (j - 1) * ds;
        double t_curr = layer_times[j];
        double t_prev = layer_times[j-1];

        // [OpenMP 修复] 只并行化外层循环 i
        #pragma omp parallel for
        for (int i = 0; i < rows; ++i) { 
            double l_curr = get_l_from_row_idx(i);
            Eigen::Vector3d end_state(l_curr, 0.0, 0.0);

            for (int k = 0; k < rows; ++k) { 
                if (dp_cost[j - 1][k] == std::numeric_limits<double>::max()) continue;

                double l_prev = get_l_from_row_idx(k);
                Eigen::Vector3d start_state(l_prev, 0.0, 0.0);

                double link_cost = CalculateLinkCost(
                    start_state, end_state, 
                    s_prev, s_curr, 
                    t_prev, t_curr,
                    obs_list, obs_pred_list, 
                    valid_max_l, valid_min_l
                );
                
                if (link_cost == std::numeric_limits<double>::max()) continue;

                double total_cost = dp_cost[j - 1][k] + link_cost; 

                if (total_cost < dp_cost[j][i]) {
                    dp_cost[j][i] = total_cost;
                    parent_index[j][i] = k;
                }
            }
        }
    }

    // --- 3. 回溯 ---
    int best_last_row = -1;
    double min_cost = std::numeric_limits<double>::max();
    for (int i = 0; i < rows; ++i) {
        if (dp_cost[cols][i] < min_cost) {
            min_cost = dp_cost[cols][i];
            best_last_row = i;
        }
    }

    if (best_last_row == -1) {
        return std::vector<Eigen::Vector4d>(); 
    }

    std::vector<int> best_path_rows;
    int curr_row = best_last_row;
    for (int j = cols; j >= 1; --j) {
        best_path_rows.push_back(curr_row);
        curr_row = parent_index[j][curr_row];
    }
    std::reverse(best_path_rows.begin(), best_path_rows.end());

    // --- 4. 生成平滑轨迹 ---
    dp_path_sl_.clear();
    dp_path_sl_.push_back(Eigen::Vector2d(fs_start.s, fs_start.l));

    double current_s = fs_start.s;
    double current_l = fs_start.l;
    double current_dl = fs_start.l_prime;
    double current_ddl = fs_start.l_double_prime;

    for (int k = 0; k < (int)best_path_rows.size(); ++k) {
        int row_idx = best_path_rows[k];
        double next_s = fs_start.s + (k + 1) * ds;
        double next_l = get_l_from_row_idx(row_idx);
        
        std::vector<double> coeffs;
        double h = next_s - current_s;
        QuinticPolynomial(0, current_l, current_dl, current_ddl,
                          h, next_l, 0.0, 0.0, coeffs); 
        
        int num_dense = (int)(h / 0.5); 
        for(int m = 1; m <= num_dense; ++m) {
            double ds_local = m * 0.5;
            double l_dense = coeffs[0] + coeffs[1]*ds_local + 
                             coeffs[2]*pow(ds_local,2) + coeffs[3]*pow(ds_local,3) + 
                             coeffs[4]*pow(ds_local,4) + coeffs[5]*pow(ds_local,5);
            dp_path_sl_.push_back(Eigen::Vector2d(current_s + ds_local, l_dense));
        }

        current_s = next_s;
        current_l = next_l;
        current_dl = 0.0; 
        current_ddl = 0.0;
    }

    // 5. 生成速度规划
    GenerateSpeedProfile(dp_path_sl_, cs_start.v, cfg_.road.velo_ref, final_traj_);

    // 6. 采样输出
    std::vector<Eigen::Vector4d> result_traj;
    for (int i = 0; i < cfg_.mpc.N; ++i) {
        double t_req = (i + 1) * cfg_.mpc.dt;
        auto it = std::lower_bound(final_traj_.begin(), final_traj_.end(), t_req, 
            [](const TrajectoryPoint& p, double val) { return p.t < val; });
        
        if (it == final_traj_.begin()) {
            result_traj.push_back(Eigen::Vector4d(it->x, it->y, it->v, it->theta));
        } else if (it == final_traj_.end()) {
            result_traj.push_back(Eigen::Vector4d(final_traj_.back().x, final_traj_.back().y, final_traj_.back().v, final_traj_.back().theta));
        } else {
            auto prev = it - 1;
            double dt = it->t - prev->t;
            if(dt < 1e-4) dt = 1e-4;
            double ratio = (t_req - prev->t) / dt;
            
            double x = prev->x + ratio * (it->x - prev->x);
            double y = prev->y + ratio * (it->y - prev->y);
            double v = prev->v + ratio * (it->v - prev->v);
            
            double d_theta = it->theta - prev->theta;
            while(d_theta > M_PI) d_theta -= 2*M_PI;
            while(d_theta < -M_PI) d_theta += 2*M_PI;
            double theta = prev->theta + ratio * d_theta;
            
            result_traj.push_back(Eigen::Vector4d(x, y, v, theta));
        }
    }
    return result_traj;
}

// void DPPlanner::QuinticPolynomial(
//     double xs, double ys, double dys, double ddys,
//     double xe, double ye, double dye, double ddye,
//     std::vector<double>& a) 
// {
//     double h = xe - xs;
//     if (std::abs(h) < 1e-4) { a.assign(6, 0.0); return; }

//     Eigen::MatrixXd A(6, 6);
//     A << 1, 0, 0, 0, 0, 0,
//          0, 1, 0, 0, 0, 0,
//          0, 0, 2, 0, 0, 0,
//          1, h, pow(h,2), pow(h,3), pow(h,4), pow(h,5),
//          0, 1, 2*h, 3*pow(h,2), 4*pow(h,3), 5*pow(h,4),
//          0, 0, 2, 6*h, 12*pow(h,2), 20*pow(h,3);

//     Eigen::VectorXd b(6);
//     b << ys, dys, ddys, ye, dye, ddye;

//     Eigen::VectorXd res = A.colPivHouseholderQr().solve(b);
//     a.resize(6);
//     for(int i=0; i<6; ++i) a[i] = res(i);
// }

// 替换原有的 QuinticPolynomial 实现
// 直接计算系数 a3, a4, a5 (a0, a1, a2 就是 start 的状态)
void DPPlanner::QuinticPolynomial(
    double xs, double ys, double dys, double ddys,
    double xe, double ye, double dye, double ddye,
    std::vector<double>& a) 
{
    double T = xe - xs;
    if (std::abs(T) < 1e-4) { a.assign(6, 0.0); return; }
    
    double T2 = T * T;
    double T3 = T2 * T;

    // 根据边界条件推导出的解析解
    // y(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    a.resize(6);
    a[0] = ys;
    a[1] = dys;
    a[2] = ddys / 2.0;

    double c1 = ye - (a[0] + a[1] * T + a[2] * T2);
    double c2 = dye - (a[1] + 2.0 * a[2] * T);
    double c3 = ddye - (2.0 * a[2]);

    // 构建 3x3 矩阵的逆的解析形式来解 a3, a4, a5
    // [ T^3   T^4    T^5 ] [a3]   [c1]
    // [3T^2  4T^3   5T^4 ] [a4] = [c2]
    // [6T    12T^2  20T^3] [a5]   [c3]
    
    // 简化后的公式：
    a[3] = (10 * c1 - 4 * c2 * T + 0.5 * c3 * T2) / T3;
    a[4] = (-15 * c1 + 7 * c2 * T - c3 * T2) / (T2 * T2); // T^4
    a[5] = (6 * c1 - 3 * c2 * T + 0.5 * c3 * T2) / (T2 * T3); // T^5
}

double DPPlanner::CalculateLinkCost(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    double start_s, double end_s,
    double start_time, double end_time, 
    const std::vector<Obstacle>& obs_list,
    const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list, 
    double road_left_bound, 
    double road_right_bound
) 
{
    std::vector<double> coeffs;
    QuinticPolynomial(0, start(0), start(1), start(2),
                      end_s - start_s, end(0), end(1), end(2), coeffs);
    
    double cost_smooth = 0.0;
    double cost_ref = 0.0;
    double cost_collision = 0.0;

    int num_samples = 10; 
    double ds_step = (end_s - start_s) / num_samples;
    double dt_step = (end_time - start_time) / num_samples; 

    for (int i = 0; i <= num_samples; ++i) {
        double s_curr = i * ds_step; 
        double t_curr = start_time + i * dt_step; 

        double l = coeffs[0] + coeffs[1]*s_curr + coeffs[2]*pow(s_curr,2) + 
                   coeffs[3]*pow(s_curr,3) + coeffs[4]*pow(s_curr,4) + coeffs[5]*pow(s_curr,5);
        
        if (l > road_left_bound + 0.01 || l < road_right_bound - 0.01) {
            return std::numeric_limits<double>::max();
        }

        double dl = coeffs[1] + 2*coeffs[2]*s_curr + 3*coeffs[3]*pow(s_curr,2) + 
                    4*coeffs[4]*pow(s_curr,3) + 5*coeffs[5]*pow(s_curr,4);
        double ddl = 2*coeffs[2] + 6*coeffs[3]*s_curr + 12*coeffs[4]*pow(s_curr,2) + 
                     20*coeffs[5]*pow(s_curr,3);
        double dddl = 6*coeffs[3] + 24*coeffs[4]*s_curr + 60*coeffs[5]*pow(s_curr,2);

        cost_smooth += cfg_.dp.w_smooth_dl * dl * dl +
                       cfg_.dp.w_smooth_ddl * ddl * ddl + 
                       cfg_.dp.w_smooth_dddl * dddl * dddl;
        cost_ref += cfg_.dp.w_ref * l * l;
        
        cost_collision += CalcObstacleCost(start_s + s_curr, l, t_curr, obs_list, obs_pred_list);
    }

    return cost_smooth + cost_ref + cost_collision;
}

double DPPlanner::CalcObstacleCost(
    double s, double l, double time, 
    const std::vector<Obstacle>& obs_list,
    const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list) 
{
    double total_cost = 0.0;
    FrenetState fs; fs.s = s; fs.l = l; fs.l_prime=0; fs.l_double_prime=0; fs.s_dot=0; fs.l_dot=0; fs.s_ddot=0; fs.l_ddot=0;
    CartesianState cs = st_.FrenetToCartesian(fs);
    Eigen::Vector2d ego_pos(cs.x, cs.y);

    double dt = cfg_.mpc.dt;     
    // [时空膨胀搜索窗口]
    int time_steps_to_check[] = {-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5}; 

    for (size_t i = 0; i < obs_list.size(); ++i) {
        if (i >= obs_pred_list.size()) continue; 
        if (obs_pred_list[i].empty()) continue;

        // [新增] 快速距离剪枝
        // 如果当前自车点距离障碍物当前位置超过 20米(举例)，直接跳过复杂的 Time Buffer 计算
        // 假设障碍物最大速度 30m/s, time buffer 0.5s -> 移动 15m. 加上车长余量。
        Eigen::Vector2d obs_curr_pos = obs_pred_list[i][0].head(2);
        if ((ego_pos - obs_curr_pos).squaredNorm() > 900.0) { // 30^2 = 900
            continue; 
        }

        double min_dist = 1e9;
        
        // 1. [动态检测]：检查预计到达时刻前后的障碍物位置
        int base_idx = std::floor(time / dt);
        for (int offset : time_steps_to_check) {
            int idx = base_idx + offset;
            if (idx < 0) idx = 0;
            if (idx >= (int)obs_pred_list[i].size()) idx = obs_pred_list[i].size() - 1;
            
            Eigen::Vector2d obs_pos = obs_pred_list[i][idx].head(2);
            double dist = (ego_pos - obs_pos).norm();
            if (dist < min_dist) min_dist = dist;
        }

        // 2. [静态兜底检测]
        if (!obs_pred_list[i].empty()) {
            Eigen::Vector2d current_obs_pos = obs_pred_list[i][0].head(2);
            double dist_current = (ego_pos - current_obs_pos).norm();
            if (dist_current < min_dist) min_dist = dist_current;
        }

        // [阈值放大]
        double threshold = 3.0; 
        if (min_dist < threshold) {
            total_cost += cfg_.dp.w_collision / (min_dist * min_dist + 0.01);
        }
    }
    return total_cost;
}

void DPPlanner::GenerateSpeedProfile(
    const std::vector<Eigen::Vector2d>& path_sl, 
    double start_v, 
    double target_v,
    std::vector<TrajectoryPoint>& out_traj) 
{
    out_traj.clear();
    if (path_sl.size() < 2) return;

    for (const auto& p : path_sl) {
        FrenetState fs; fs.s = p.x(); fs.l = p.y(); fs.l_prime = 0; fs.l_double_prime = 0; fs.s_dot = 0; fs.l_dot=0; fs.s_ddot=0; fs.l_ddot=0;
        CartesianState cs = st_.FrenetToCartesian(fs);
        
        TrajectoryPoint tp;
        tp.x = cs.x; tp.y = cs.y; tp.theta = cs.theta; tp.kappa = cs.kappa;
        tp.s = p.x(); tp.l = p.y();
        out_traj.push_back(tp);
    }

    double max_acc = cfg_.vehicle.a_max;
    double max_dec = std::abs(cfg_.vehicle.a_min);
    double max_lat_acc = 2.0;

    for (auto& p : out_traj) {
        double kappa = std::abs(p.kappa);
        double v_lim_lat = std::sqrt(max_lat_acc / (kappa + 1e-3));
        p.v = std::min(target_v, v_lim_lat);
        p.v = std::min(p.v, cfg_.vehicle.velo_max);
    }
    out_traj[0].v = start_v;

    for (size_t i = 1; i < out_traj.size(); ++i) {
        double ds = std::hypot(out_traj[i].x - out_traj[i-1].x, out_traj[i].y - out_traj[i-1].y);
        double v_max_reachable = std::sqrt(out_traj[i-1].v * out_traj[i-1].v + 2 * max_acc * ds);
        if (out_traj[i].v > v_max_reachable) out_traj[i].v = v_max_reachable;
    }

    for (int i = out_traj.size() - 2; i >= 0; --i) {
        double ds = std::hypot(out_traj[i+1].x - out_traj[i].x, out_traj[i+1].y - out_traj[i].y);
        double v_max_braking = std::sqrt(out_traj[i+1].v * out_traj[i+1].v + 2 * max_dec * ds);
        if (out_traj[i].v > v_max_braking) out_traj[i].v = v_max_braking;
    }

    out_traj[0].t = 0.0;
    for (size_t i = 1; i < out_traj.size(); ++i) {
        double ds = std::hypot(out_traj[i].x - out_traj[i-1].x, out_traj[i].y - out_traj[i-1].y);
        double v_avg = (out_traj[i].v + out_traj[i-1].v) / 2.0;
        double dt = ds / std::max(v_avg, 0.1);
        out_traj[i].t = out_traj[i-1].t + dt;
    }
}