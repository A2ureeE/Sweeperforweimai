#include "Visualizer.h"
#include "matplotlibcpp.h"
#include "CILQR.h" 
#include "Utils.h" 
#include <cmath>
#include <iostream>
#include <iomanip> 
#include <sstream>
#include <algorithm> 
#include <limits>
#include <thread> 
#include <chrono> 

namespace plt = matplotlibcpp;

Visualizer::Visualizer(const Config& config) : cfg(config) {
    // 强制设置交互式后端
    try {
        plt::backend("TkAgg"); 
    } catch (...) {}
    // 初始化画布大小并开启交互模式 (ion)
    try {
        plt::figure_size(1200, 800);
        plt::ion(); 
    } catch (...) {}
}

bool is_valid(double val) { return std::isfinite(val) && !std::isnan(val); }

void safe_plot(const std::vector<double>& x, const std::vector<double>& y, const std::map<std::string, std::string>& keywords) {
    if (x.empty() || x.size() != y.size()) return;
    std::vector<double> sx, sy;
    sx.reserve(x.size()); sy.reserve(y.size());
    for(size_t i=0; i<x.size(); ++i) {
        if(is_valid(x[i]) && is_valid(y[i])) {
            sx.push_back(x[i]); sy.push_back(y[i]);
        }
    }
    if(sx.size() > 1) {
        try { plt::plot(sx, sy, keywords); } catch(...) {}
    }
}

// 仿真结束后的详细分析绘图实现
void Visualizer::plot_analysis(const AnalysisData& data) {
    std::cout << "[Visualizer] Generating analysis figures..." << std::endl;
    // 2. 执行 plt.close('all') 关闭之前的仿真动画窗口，清理内存。
    PyRun_SimpleString("import matplotlib.pyplot as plt; plt.ioff(); plt.close('all');");

    try {
        // --- Figure 1: 速度分析 ---
        plt::figure(1); 
        // plt::clf(); // 已经在 close('all') 中清理过，这里可以省略，或者保留以防万一
        plt::title("Analysis: Speed Tracking");
        safe_plot(data.time, data.speed, {{"label", "Actual Speed (m/s)"}, {"color", "blue"}, {"linewidth", "2"}});
        safe_plot(data.time, data.ref_speed, {{"label", "Ref Speed"}, {"color", "green"}, {"linestyle", "--"}});
        plt::ylabel("Speed [m/s]");
        plt::xlabel("Time [s]");
        plt::legend();
        plt::grid(true);

        // --- Figure 2: 加速度分析 ---
        plt::figure(2);
        plt::title("Analysis: Acceleration Control");
        safe_plot(data.time, data.accel, {{"label", "Acceleration (m/s^2)"}, {"color", "red"}});
        plt::ylabel("Accel [m/s^2]");
        plt::xlabel("Time [s]");
        plt::legend();
        plt::grid(true);

        // --- Figure 3: 转向角分析 ---
        plt::figure(3);
        plt::title("Analysis: Steering Angle");
        // 将弧度转换为度
        std::vector<double> steer_deg;
        for(double s : data.steer) steer_deg.push_back(s * 180.0 / M_PI);
        safe_plot(data.time, steer_deg, {{"label", "Steering Angle (deg)"}, {"color", "purple"}, {"linewidth", "2"}});
        plt::ylabel("Steer [deg]");
        plt::xlabel("Time [s]");
        plt::legend();
        plt::grid(true);

        // --- Figure 4: 转向速率分析 ---
        plt::figure(4);
        plt::title("Analysis: Steering Rate");
        std::vector<double> rate_deg;
        for(double r : data.steer_rate) rate_deg.push_back(r * 180.0 / M_PI);
        safe_plot(data.time, rate_deg, {{"label", "Steering Rate (deg/s)"}, {"color", "orange"}});
        plt::ylabel("Steer Rate [deg/s]");
        plt::xlabel("Time [s]");
        plt::legend();
        plt::grid(true);

        // --- Figure 5: 横向误差分析 ---
        plt::figure(5);
        plt::title("Analysis: Lateral Tracking Error");
        safe_plot(data.time, data.lat_error, {{"label", "Lateral Error (m)"}, {"color", "black"}});
        // 绘制零线
        std::vector<double> zero_line(data.time.size(), 0.0);
        safe_plot(data.time, zero_line, {{"color", "gray"}, {"linestyle", "--"}});
        plt::ylabel("Lateral Error [m]");
        plt::xlabel("Time [s]");
        plt::legend();
        plt::grid(true);

        std::cout << "[Visualizer] Plots ready. Please close all figure windows to exit the program." << std::endl;
        
        // 阻塞显示
        plt::show(true);
        
    } catch (const std::runtime_error& e) {
        std::cerr << "[Visualizer Error] Plotting failed: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[Visualizer Error] Unknown error during plotting." << std::endl;
    }
}


void Visualizer::draw(const Eigen::Vector4d& ego_state, 
          const Eigen::Vector2d& current_u,
          double ref_speed,
          const std::vector<Obstacle>& obs_list,
          const std::vector<Eigen::Vector4d>& plan_x,// CILQR 结果轨迹
          const std::vector<Eigen::Vector4d>& dp_trajectory,// DP 参考轨迹
          const PathData& path_data,// 地图数据
          const std::vector<Eigen::Vector4d>& history_ego) // 历史轨迹
{
    if (!is_valid(ego_state.sum())) return; // 状态无效则跳过

    try { plt::clf(); } catch(...) { return; } // 状态无效则跳过

    // --- 1. 计算视野范围 ---
    // 遍历所有地图边界点，确定当前绘图的 XY 限制
    // 如果地图数据为空，则以自车为中心生成默认范围
    double gx_min = std::numeric_limits<double>::max();
    double gx_max = std::numeric_limits<double>::lowest();
    double gy_min = std::numeric_limits<double>::max();
    double gy_max = std::numeric_limits<double>::lowest();
    bool has_bounds = false;

    auto update_bounds = [&](const std::vector<Eigen::Vector2d>& pts) {
        for(const auto& p : pts) {
            if(is_valid(p.x()) && is_valid(p.y())) {
                if(p.x() < gx_min) gx_min = p.x();
                if(p.x() > gx_max) gx_max = p.x();
                if(p.y() < gy_min) gy_min = p.y();
                if(p.y() > gy_max) gy_max = p.y();
                has_bounds = true;
            }
        }
    };
    update_bounds(path_data.outer_boundary);
    update_bounds(path_data.inner_boundary);

    if (!has_bounds || gx_max <= gx_min + 1.0 || gy_max <= gy_min + 1.0) {
        double cx = ego_state(0); double cy = ego_state(1);
        gx_min = cx - 60; gx_max = cx + 60;
        gy_min = cy - 40; gy_max = cy + 40;
    }

    // --- 2. 绘制赛道/地图 ---
    // 辅助 lambda: 将 Point 结构体转为 vector<double> 供 matplotlib 使用
    auto vec_to_plot = [](const std::vector<Eigen::Vector2d>& pts, std::vector<double>& out_x, std::vector<double>& out_y) {
        out_x.clear(); out_y.clear();
        for(const auto& p : pts) { out_x.push_back(p.x()); out_y.push_back(p.y()); }
    };
    std::vector<double> px, py;

    // 2.1 绘制物理边界 (Inner/Outer Boundary) - 实线
    vec_to_plot(path_data.inner_boundary, px, py);
    safe_plot(px, py, {{"color", "black"}, {"linestyle", "-"}, {"linewidth", "1.5"}});
    vec_to_plot(path_data.outer_boundary, px, py);
    safe_plot(px, py, {{"color", "black"}, {"linestyle", "-"}, {"linewidth", "1.5"}});

    // 2.2 绘制道路几何中心线 - 虚线
    vec_to_plot(path_data.waypoints_center, px, py);
    safe_plot(px, py, {{"color", "black"}, {"linestyle", "--"}, {"linewidth", "1.2"}, {"label", "Center"}});

    // 2.3 绘制车道线 (如内侧车道中心线) - 点状线
    // 确保 path_data.waypoints_inner 有数据
    if (!path_data.waypoints_inner.empty()) {
        vec_to_plot(path_data.waypoints_inner, px, py);
        safe_plot(px, py, {{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "1.0"}, {"label", "Inner"}});
    }

    // 2.4 当前激活参考线 (Outer)
    vec_to_plot(path_data.waypoints, px, py);
    safe_plot(px, py, {{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "1.0"}, {"label", "Outer"}});

    // --- 3. 绘制障碍物 ---
    for(const auto& obs : obs_list) {
        if(is_valid(obs.state.sum())) {
            draw_vehicle(obs.state, "r", obs.length, obs.width);
            draw_obstacle_ellipse(obs);
        }
    }

    // --- 4. 绘制轨迹 ---
    // 4.1 DP 生成的粗略参考轨迹 (Magenta/洋红色, 点划线)
    std::vector<double> dpx, dpy;
    for(const auto& p : dp_trajectory) { dpx.push_back(p(0)); dpy.push_back(p(1)); }
    safe_plot(dpx, dpy, {{"color", "magenta"}, {"linestyle", "-."}, {"linewidth", "2.0"}, {"label", "DP Ref"}});

    // 4.2 CILQR 优化后的平滑轨迹 (Cyan/青色, 实线, 粗)
    std::vector<double> cx, cy;
    for(const auto& p : plan_x) { cx.push_back(p(0)); cy.push_back(p(1)); }
    safe_plot(cx, cy, {{"color", "cyan"}, {"linestyle", "-"}, {"linewidth", "2.5"}, {"label", "CILQR Traj"}});

    // --- 5. 绘制自车 (蓝色) 及历史轨迹 (绿色) ---
    draw_vehicle(ego_state, "b", cfg.vehicle.length, cfg.vehicle.width);
    std::vector<double> hx, hy;
    for(const auto& h : history_ego) { hx.push_back(h(0)); hy.push_back(h(1)); }
    safe_plot(hx, hy, {{"color", "green"}, {"linestyle", "--"}, {"linewidth", "1.5"}});

    // --- 6. 设置视野 ---
    double w = gx_max - gx_min;
    double h = gy_max - gy_min;
    gx_min -= w * 0.05; gx_max += w * 0.05;
    gy_min -= h * 0.05; gy_max += h * 0.05;

    double cur_w = gx_max - gx_min;
    double cur_h = gy_max - gy_min;
    if (cur_w < 1.0) cur_w = 100.0;
    if (cur_h < 1.0) cur_h = 60.0;

    double ratio = 1200.0 / 800.0;
    if (cur_w / cur_h > ratio) {
        double target_h = cur_w / ratio;
        double cy = (gy_min + gy_max) / 2.0;
        gy_min = cy - target_h / 2.0;
        gy_max = cy + target_h / 2.0;
    } else {
        double target_w = cur_h * ratio;
        double cx = (gx_min + gx_max) / 2.0;
        gx_min = cx - target_w / 2.0;
        gx_max = cx + target_w / 2.0;
    }

    try {
        plt::xlim(gx_min, gx_max);
        plt::ylim(gy_min, gy_max);
    } catch (...) {}

    // --- 7. HUD (Head-Up Display) ---
    // 在画布左上角显示文字信息：速度、转向角、横向误差、距离边界距离等
    if (!path_data.waypoints.empty()) {
        try {
            // 计算自车相对于当前样条曲线的横向误差 e_y
            // (Newton 投影算法调用)
            double min_dist = 1e9;
            int idx = 0;
            for(size_t j=0; j<path_data.waypoints.size(); j+=5) {
                double d = (path_data.waypoints[j] - ego_state.head(2)).norm();
                if (d < min_dist) { min_dist = d; idx = j; }
            }
            double t_proj = Utils::project_to_spline_newton(ego_state.head(2), const_cast<Spline2D&>(path_data.spline), path_data.t_params[idx]);
            SplineProps props = Utils::get_spline_properties(const_cast<Spline2D&>(path_data.spline), t_proj);
            double e_y = (ego_state.head(2) - props.pos).dot(props.normal_left);
            
            double dist_left = 9.0 - e_y;
            double dist_right = e_y - (-3.0);
            double safe_dist = std::min(dist_left, dist_right) - cfg.vehicle.width/2.0;
            double limit = std::max(0.0, cfg.road.width/2.0 - cfg.mpc.road_safe_margin); // 粗略估计限制

            // 详细 HUD
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2);
            ss << "=== Vehicle State ===\n";
            ss << "Pos: (" << ego_state(0) << ", " << ego_state(1) << ")\n";
            ss << "Speed: " << ego_state(2)*3.6 << " km/h\n";
            ss << "Accel: " << current_u(0) << " m/s^2\n";
            ss << "Steer: " << (current_u(1) * 180.0 / M_PI) << " deg\n";
            ss << "\n=== Road Status ===\n";
            ss << "Lat Error: " << e_y << " m\n";
            ss << "Dist to Bound: " << safe_dist << " m\n";
            ss << "Status: " << (safe_dist < 0 ? "VIOLATION!" : "OK");
            
            double tx = gx_min + (gx_max - gx_min) * 0.02;
            double ty = gy_max - (gy_max - gy_min) * 0.20; // 留出足够高度
            if (is_valid(tx) && is_valid(ty)) plt::text(tx, ty, ss.str());
        } catch(...) {}
    }

    try {
        plt::pause(0.01); 
    } catch (const std::exception& e) {
        try {
            plt::show(false);
            plt::draw();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } catch(...) {}
    }
}

// 绘制单个车辆矩形
void Visualizer::draw_vehicle(const Eigen::Vector4d& state, std::string color, double length, double width) {
    // 1. 定义矩形四个角点 (车体坐标系)
    // 2. 使用旋转矩阵 R(yaw) 将其转换到世界坐标系
    // 3. 平移到车辆当前位置 (x, y)
    // 4. 绘图
    if (!is_valid(state.sum())) return;
    double x = state(0); double y = state(1); double yaw = state(3);
    Eigen::MatrixXd corners(2, 5);
    corners << -length/2, length/2, length/2, -length/2, -length/2, -width/2, -width/2, width/2, width/2, -width/2;    
    Eigen::Matrix2d rot; rot << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    corners = rot * corners;
    std::vector<double> vx, vy;
    for(int i=0; i<5; ++i) { vx.push_back(corners(0,i) + x); vy.push_back(corners(1,i) + y); }
    safe_plot(vx, vy, {{"color", color}});
}

// 绘制障碍物安全椭圆
void Visualizer::draw_obstacle_ellipse(const Obstacle& obs) {
    if (!is_valid(obs.state.sum())) return;
    double x = obs.state(0); double y = obs.state(1); double yaw = obs.state(3);
    double ego_radius = cfg.vehicle.width / 2.0; 
    double a, b;
    // 根据 Utils 中的计算公式绘制椭圆边界
    Utils::get_ellipsoid_scales(ego_radius, obs.width, obs.length, obs.d_safe, a, b);
    std::vector<double> ex, ey;
    for (int i = 0; i <= 30; ++i) {
        double t = 2.0 * M_PI * i / 30;
        double lx = a * cos(t); double ly = b * sin(t);
        ex.push_back(x + lx * cos(yaw) - ly * sin(yaw));
        ey.push_back(y + lx * sin(yaw) + ly * cos(yaw));
    }
    safe_plot(ex, ey, {{"color", "red"}, {"linestyle", "--"}, {"linewidth", "0.8"}});
}