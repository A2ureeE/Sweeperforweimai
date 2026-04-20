#include "Config.h"
#include "Utils.h"
#include "CILQR.h"
#include "Visualizer.h"
#include "Dynamics.h"
#include "StateTransformer.h" 
#include "DPPlanner.h"        
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm> 
#include <iomanip> 

/**
 * @brief 检查状态向量是否有效 (不含 NaN 或 Inf)
 * @param state 5维状态向量 [x, y, v, yaw, steer]
 * @return true 如果有效, false 否则
 */
bool is_state_valid(const Vector5d& state) {
    return !std::isnan(state.sum()) && !std::isinf(state.sum());
}

// --- Rate Limiter (速率限制器) 相关定义 ---
// 用于模拟底层控制器的物理限制，或者作为最后一道安全防线
struct ControlCommand {
    double steer; // 弧度
    double accel; // m/s^2
};

/**
 * @brief 变化率限制器 (Rate Limiter)
 * 限制控制量的变化速率，防止由于规划器输出跳变导致的执行器过载或车辆失稳。
 * 虽然 5状态 CILQR 模型已经包含了 steer_rate 的约束，但这个模块依然有价值，
 * 它可以模拟实际转向系统的物理瓶颈。
 * * @param target 规划器输出的目标控制量
 * @param current 上一时刻的执行控制量
 * @param dt 控制周期
 * @return ControlCommand 限制后的控制量
 */
ControlCommand rate_limiter(const ControlCommand& target, const ControlCommand& current, double dt) {
    const double MAX_STEER_RATE = 200.0 * M_PI / 180.0; 
    const double MAX_ACCEL_JERK = 5.0; 

    ControlCommand cmd;
    
    // 1. 限制转向变化率
    double d_steer = target.steer - current.steer;
    double max_d_steer = MAX_STEER_RATE * dt;
    
    if (d_steer > max_d_steer) d_steer = max_d_steer;
    else if (d_steer < -max_d_steer) d_steer = -max_d_steer;
    
    cmd.steer = current.steer + d_steer;

    // 2. 限制加速度变化率 (Jerk)
    double d_accel = target.accel - current.accel;
    double max_d_accel = MAX_ACCEL_JERK * dt;
    
    if (d_accel > max_d_accel) d_accel = max_d_accel;
    else if (d_accel < -max_d_accel) d_accel = -max_d_accel;
    
    cmd.accel = current.accel + d_accel;

    return cmd;
}
// --------------------------------

int main() {
    // 1. 加载全局配置
    Config config = Config::getConfig();
    std::cout << "Initializing CILQR + DP Planner (Complex Traffic Scenario)..." << std::endl;

    // 2. 生成静态地图与参考线
    // 这里生成了包含中心线、内侧车道、外侧车道的完整赛道数据
    // static_path_data.spline 默认对应外侧车道 (Outer Lane)
    PathData static_path_data = Utils::generate_racetrack_data(config.track, config.road.width);
    
    // 3. 初始化各模块
    StateTransformer st(static_path_data.spline); // 坐标转换器 (Cartesian <-> Frenet)
    DPPlanner dp_planner(config, st); // 动态规划粗规划器
    Visualizer vis(config);           // 可视化模块
    Utils utils; 

    // 单独构建内侧车道的 Spline，用于生成位于内侧车道的动态障碍物
    Spline2D inner_spline;
    {
        std::vector<double> ix, iy;
        for(const auto& p : static_path_data.waypoints_inner) {
            ix.push_back(p.x());
            iy.push_back(p.y());
        }
        inner_spline.init(ix, iy);
    }

    std::cout << "Generating mixed traffic flow..." << std::endl;
    std::vector<Obstacle> obs_list;
    std::vector<double> obs_s_params; // 存储每个障碍物当前的 s 坐标
    std::vector<int> obs_lane_ids;    // 存储每个障碍物所在的车道 ID (0: Outer, 1: Inner)

    // --- 生成外侧车道障碍物 ---
    int num_obs_outer = 8;
    int start_offset_outer = 100;
    for (int i = 0; i < num_obs_outer; ++i) {
        int total_wp = static_path_data.waypoints_outer.size();
        int idx = (start_offset_outer + i * (total_wp / num_obs_outer)) % total_wp;
        Eigen::Vector2d p = static_path_data.waypoints_outer[idx];
        double t_guess = static_path_data.t_params[idx];
        // 将路点投影到 Spline 上获取精确的 s
        double s_init = Utils::project_to_spline_newton(p, static_path_data.spline, t_guess);
        obs_s_params.push_back(s_init);
        obs_lane_ids.push_back(0); 

        Obstacle obs;
        obs.wheelbase = 2.9; obs.width = 1.8; obs.length = 4.5; obs.d_safe = 0.8;
        // 前两个设为动态障碍物，其余为静态
        double speed = (i < 3) ? 3.0 : 0.0; 
        obs.type = (i < 3) ? "dynamic" : "static";
        obs.state = Utils::get_state_from_spline(static_path_data.spline, s_init, speed);
        obs_list.push_back(obs);
    }

    // --- 生成内侧车道障碍物 (Inner Lane Obstacles) ---
    int num_obs_inner = 8;
    int start_offset_inner = 50; 
    for (int i = 0; i < num_obs_inner; ++i) {
        int total_wp = static_path_data.waypoints_inner.size();
        int idx = (start_offset_inner + i * (total_wp / num_obs_inner)) % total_wp;
        Eigen::Vector2d p = static_path_data.waypoints_inner[idx];
        double s_guess = (double)idx / total_wp * inner_spline.total_length;
        double s_init = Utils::project_to_spline_newton(p, inner_spline, s_guess);
        obs_s_params.push_back(s_init);
        obs_lane_ids.push_back(1); 

        Obstacle obs;
        obs.wheelbase = 2.9; obs.width = 1.8; obs.length = 4.5; obs.d_safe = 0.8;
        double speed = (i < 3) ? 4.0 : 0.0; 
        obs.type = (i < 3) ? "dynamic" : "static";
        obs.state = Utils::get_state_from_spline(inner_spline, s_init, speed);
        obs_list.push_back(obs);
    }

    // CILQR 规划器实例
    CILQR planner(config);
    // 初始化自车状态 [x, y, v, yaw, steer] (5状态)
    Vector5d current_ego_state;
    current_ego_state << static_path_data.waypoints_outer[0].x(), 
                         static_path_data.waypoints_outer[0].y(), 
                         0.0, -M_PI/2.0, 0.0; 
    // 初始化 Warm Start 变量
    // prev_opti_x: 上一帧优化得到的状态轨迹 (5D)
    // prev_opti_u: 上一帧优化得到的控制序列
    std::vector<Vector5d> prev_opti_x;
    std::vector<Eigen::Vector2d> prev_opti_u;
    prev_opti_u.assign(config.mpc.N, Eigen::Vector2d::Zero());

    std::vector<Vector5d> history_ego;// 记录历史轨迹用于回放/绘图
    ControlCommand last_exec_cmd = {0.0, 0.0}; 

    // 用于存储分析数据的容器
    AnalysisData log_data;
    // 预分配内存，避免 vector 频繁扩容
    log_data.time.reserve(3000);
    log_data.speed.reserve(3000);
    log_data.ref_speed.reserve(3000);
    log_data.accel.reserve(3000);
    log_data.steer.reserve(3000);
    log_data.steer_rate.reserve(3000);
    log_data.lat_error.reserve(3000);
    log_data.yaw.reserve(3000);

    // 记录上一次用于日志记录的投影S值，防止投影跳变
    // 这是一个优化技巧：利用时序连贯性，下次投影搜索从上次结果附近开始
    double last_proj_s_log = 0.0;

    int MAX_STEPS = 3000;

    // --- 主循环 ---
    for (int step = 0; step < MAX_STEPS; ++step) {
        auto start_time = std::chrono::high_resolution_clock::now();
        double current_time = step * config.mpc.dt;
        
        // --- 1. 障碍物轨迹预测 (Prediction) ---
        // 对每个障碍物，基于其当前状态和所在车道的几何信息，预测未来 N 步的轨迹
        std::vector<std::vector<Eigen::Vector4d>> obs_pred_list(obs_list.size());
        for (size_t i = 0; i < obs_list.size(); ++i) {
            double current_s = obs_s_params[i];
            double speed = obs_list[i].state(2); 
            if (obs_list[i].type == "dynamic") {
                // 动态障碍物：沿当前车道线匀速运动
                Spline2D& target_spline = (obs_lane_ids[i] == 0) ? static_path_data.spline : inner_spline;
                obs_pred_list[i] = Utils::predict_obstacle_trajectory(
                    current_s, speed, target_spline, config.mpc.N, config.mpc.dt);
            } else {
                // 静态障碍物：位置保持不变
                obs_pred_list[i].assign(config.mpc.N + 1, obs_list[i].state);
            }
        }

        // --- 2. 动态规划 (DP Planner) ---
        // 在 Frenet (s-l) 空间搜索一条无碰撞的粗略路径
        double road_relative_left = config.road.width * 3.0 / 4.0;
        double road_relative_right = -config.road.width / 4.0;       
        // DP 接口需要 Vector4d [x,y,v,yaw]，我们传入 5D 状态的前 4 维
        std::vector<Eigen::Vector4d> dp_traj_cartesian = dp_planner.Plan(
            current_ego_state.head(4), obs_list, obs_pred_list, road_relative_left, road_relative_right
        );
        // 根据 DP 结果构建动态参考路径
        PathData dynamic_ref_path = static_path_data; 
        std::vector<double> dp_x, dp_y;
        bool dp_success = !dp_traj_cartesian.empty();

        if (dp_success) {
            // 如果 DP 成功，用 DP 的结果拟合一条新的 Spline 作为 CILQR 的参考线
            // 这样 CILQR 的任务就是“平滑地跟踪 DP 路径”
            for(const auto& p : dp_traj_cartesian) {
                dp_x.push_back(p(0));
                dp_y.push_back(p(1));
            }
            dynamic_ref_path.spline.init(dp_x, dp_y);
            dynamic_ref_path.t_params = dynamic_ref_path.spline.s;
            dynamic_ref_path.waypoints.clear();
            for(size_t k=0; k<dp_x.size(); ++k) {
                dynamic_ref_path.waypoints.push_back(Eigen::Vector2d(dp_x[k], dp_y[k]));
            }
        } else {
            // 如果 DP 失败 (极其罕见，除非死路)，回退到静态地图中心线
            // 这是一个 Failure Mode 处理
            dynamic_ref_path = static_path_data;
        }

        // --- 3. 执行 CILQR 优化 (Local Planner) ---
        // 基于 5 状态动力学模型，优化控制量序列
        std::vector<Eigen::Vector2d> opti_u;
        std::vector<Vector5d> opti_x; // 5D
        const std::vector<TrajectoryPoint>& dp_full_traj = dp_planner.GetDetailedTrajectory();

        planner.solve(current_ego_state, utils, dynamic_ref_path, 
                      dp_full_traj, // DP 提供的参考速度和位置
                      obs_list, obs_pred_list, prev_opti_x, prev_opti_u,
                      opti_u, opti_x);
        
        // 4. 获取控制量
        Eigen::Vector2d target_u;
        if (!opti_u.empty() && !std::isnan(opti_u[0].sum())) {
            target_u = opti_u[0];
        } else {
            // 紧急回退控制：最大制动
            target_u << -3.0, 0.0; 
        }

        Eigen::Vector2d exec_u = target_u; // 最终执行的控制量 [acc, steer_rate]

        // 5. Rate Limiter (可选保留)
        // 在 5 状态模型中，exec_u(1) 已经是 steer_rate，且受约束限制。
        // 这里可以根据实际需要决定是否启用额外的 limiter。
        // 为了展示完整性，保留此结构，但实际上 CILQR 输出已平滑。
        ControlCommand target_cmd = {0.0, exec_u(0)}; 
        // 5状态模型本身已对 steer_rate 进行约束，这里仅作为最后的安全网
        
        double current_steer_rate = exec_u(1); 

        
        // --- 6. 数据记录 (用于分析) ---
        // 计算真实的横向误差 (相对于静态参考线，而非 DP 路径)
        // 这样可以反映出车辆为了避障偏离了车道中心多少米
        // 使用 last_proj_s_log 作为初值，保证找到赛道上真正的最近点
        double t_proj = Utils::project_to_spline_newton(current_ego_state.head(2), static_path_data.spline, last_proj_s_log); 
        last_proj_s_log = t_proj; // 更新缓存

        SplineProps props = Utils::get_spline_properties(static_path_data.spline, t_proj);
        double e_y = (current_ego_state.head(2) - props.pos).dot(props.normal_left);

        // 7. 记录数据
        log_data.time.push_back(current_time);
        log_data.speed.push_back(current_ego_state(2));
        log_data.ref_speed.push_back(config.road.velo_ref);
        log_data.accel.push_back(exec_u(0));
        log_data.steer.push_back(current_ego_state(4)); 
        log_data.steer_rate.push_back(current_steer_rate);
        log_data.lat_error.push_back(e_y);
        log_data.yaw.push_back(current_ego_state(3));

        // --- 8. 更新自车状态 (Simulation Step) ---
        // 使用高精度 RK4 积分器推进车辆状态
        current_ego_state = Dynamics::bicycle_model(current_ego_state, exec_u, config.mpc.dt, config.vehicle.wheelbase);
        // 状态有效性检查
        if (!is_state_valid(current_ego_state)) {
            current_ego_state = opti_x.empty() ? prev_opti_x[0] : opti_x[0];
        }
        // 更新 Warm Start 缓存
        prev_opti_x = opti_x;
        prev_opti_u = opti_u;
        history_ego.push_back(current_ego_state);

        // 9. 更新障碍物
        for (size_t i = 0; i < obs_list.size(); ++i) {
            if (obs_list[i].type == "dynamic") {
                double v = obs_list[i].state(2);
                Spline2D& target_spline = (obs_lane_ids[i] == 0) ? static_path_data.spline : inner_spline;
                // 简单的沿样条曲线运动模型
                double s_max = target_spline.get_s_max();
                double s_min = target_spline.get_s_min();
                double track_len = s_max - s_min;
                obs_s_params[i] += v * config.mpc.dt;
                // 处理赛道闭环
                obs_s_params[i] = fmod(obs_s_params[i] - s_min, track_len);
                if (obs_s_params[i] < 0) obs_s_params[i] += track_len;
                obs_s_params[i] += s_min;
                obs_list[i].state = Utils::get_state_from_spline(target_spline, obs_s_params[i], v);
            }
        }

        // --- 10. 实时可视化 ---
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

        if (step % 2 == 0) { // 降频显示，避免拖慢仿真
             std::cout << std::fixed << std::setprecision(2)
                       << "Step: " << std::setw(4) << step 
                       << " | Time: " << std::setw(5) << elapsed.count() << "ms"
                       << " | Spd: " << std::setw(5) << current_ego_state(2)*3.6 << "km/h"
                       << " | Steer: " << std::setw(5) << current_ego_state(4) * 180.0 / M_PI << "deg"
                       << " | Rate: " << std::setw(5) << exec_u(1) * 180.0 / M_PI << "d/s"
                       << std::endl;
            
             // 构造用于显示的控制向量 [acc, steer_angle] (适配 Visualizer 接口)
             Eigen::Vector2d display_u; 
             display_u << exec_u(0), current_ego_state(4); 
            
             // 转换数据格式用于绘图 (5D -> 4D)
             std::vector<Eigen::Vector4d> opti_x_4d;
             for(const auto& s : opti_x) opti_x_4d.push_back(s.head(4));
             
             std::vector<Eigen::Vector4d> history_ego_4d;
             for(const auto& s : history_ego) history_ego_4d.push_back(s.head(4));

             vis.draw(current_ego_state.head(4), display_u, config.road.velo_ref, 
                      obs_list, opti_x_4d, dp_traj_cartesian, 
                      static_path_data, 
                      history_ego_4d);
        }
    }
    
    // --- 11. 仿真结束，生成详细分析图表 ---
    std::cout << "Simulation Finished. Generating analysis plots..." << std::endl;
    vis.plot_analysis(log_data);

    return 0;
}