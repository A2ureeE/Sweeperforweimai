#include "Utils.h"
#include "Dynamics.h"
#include "StateTransformer.h" 
#include <cmath>
#include <algorithm>
#include <iostream>

// 生成赛道数据：将直线和圆弧段解析为点集，并构建 Spline
PathData Utils::generate_racetrack_data(const TrackConfig& track_def, double road_width) {
    PathData pd;
    std::vector<double> rx, ry;
    
    // --- Step 1: 生成几何中心线 ---
    auto add_point = [&](double x, double y) {
        if (!rx.empty()) {
            double dist = std::hypot(x - rx.back(), y - ry.back());
            // [Fix] 增加最小间距检查，防止重复点导致 Spline 导数爆炸
            if (dist < 0.1) return; 
        }
        rx.push_back(x);
        ry.push_back(y);
    };

    for (const auto& segment : track_def.segments) {
        if (segment.type == "arc") {
            double cx = segment.params[0];
            double cy = segment.params[1];
            double cr = segment.params[2];
            double start_ang = segment.params[3];
            double end_ang = segment.params[4];
            if (std::abs(start_ang - end_ang) < 1e-6) continue;
            
            // 使用配置中的 step_curve，现在更密集了
            double total_angle = std::abs(end_ang - start_ang);
            int steps = std::ceil(total_angle / track_def.step_curve);
            // 保证至少有足够多的点
            if(steps < 5) steps = 5; 

            for (int i = 0; i <= steps; ++i) {
                double theta = start_ang + (end_ang - start_ang) * ((double)i / steps);
                add_point(cx + cr * cos(theta), cy + cr * sin(theta));
            }
        } else if (segment.type == "line") {
            double x1 = segment.params[0];
            double y1 = segment.params[1];
            double x2 = segment.params[2];
            double y2 = segment.params[3];
            double dist = std::hypot(x2 - x1, y2 - y1);
            if (dist < 1e-6) continue;
            
            int num_steps = std::ceil(dist / track_def.step_line);
            if(num_steps < 2) num_steps = 2;

            for (int i = 0; i < num_steps; ++i) { // 注意这里 < num_steps，避免与下一段起点重复
                double t = (double)i / num_steps;
                add_point(x1 + t * (x2 - x1), y1 + t * (y2 - y1));
            }
        }
    }

    // [Fix] 确保闭合，同时去除末尾极其接近起点的点
    if (!rx.empty()) {
        double close_dist = std::hypot(rx.back() - rx[0], ry.back() - ry[0]);
        // 如果末尾点离起点太近但不是同一点，移除它，让 init_periodic 处理闭合
        if (close_dist < 0.5 && rx.size() > 2) {
            rx.pop_back();
            ry.pop_back();
        }
        // 对于 periodic spline，我们通常显式添加起点到末尾，或者让库处理
        // Spline.cpp 的实现需要起点和终点重合来计算周期性，这里显式闭合
        rx.push_back(rx[0]);
        ry.push_back(ry[0]);
    }

    pd.spline.init(rx, ry);

    // --- Step 2: 生成多车道数据 ---
    StateTransformer st(pd.spline);

    int num_pts = track_def.num_smooth_points;
    double s_min = pd.spline.get_s_min();
    double s_max = pd.spline.get_s_max();
    double ds = (s_max - s_min) / (num_pts - 1);

    double l_ref_center = 0.0;
    double l_ref_inner  =  road_width / 4.0; 
    double l_ref_outer  = -road_width / 4.0; 
    
    double l_bound_physical_inner =  road_width / 2.0; 
    double l_bound_physical_outer = -road_width / 2.0; 

    for (int i = 0; i < num_pts; ++i) {
        double s = s_min + i * ds;
        if (s > s_max) s = s_max; 
        
        FrenetState fs; 
        fs.s = s; fs.s_dot = 0; fs.l_dot = 0; fs.s_ddot = 0; fs.l_ddot = 0; fs.l_prime = 0; fs.l_double_prime = 0;

        // 1. Center Line
        fs.l = l_ref_center;
        pd.waypoints_center.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});
        fs.l = l_bound_physical_inner;
        pd.waypoints_center_inner_boundary.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});
        fs.l = l_bound_physical_outer;
        pd.waypoints_center_outer_boundary.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});

        // 2. Outer Line (CCW)
        fs.l = l_ref_outer;
        pd.waypoints_outer.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});
        fs.l = l_bound_physical_inner;
        pd.waypoints_outer_inner_boundary.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});
        fs.l = l_bound_physical_outer;
        pd.waypoints_outer_outer_boundary.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});

        // 3. Inner Line (CW)
        fs.l = l_ref_inner;
        pd.waypoints_inner.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});
        fs.l = l_bound_physical_inner;
        pd.waypoints_inner_inner_boundary.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});
        fs.l = l_bound_physical_outer;
        pd.waypoints_inner_outer_boundary.push_back({st.FrenetToCartesian(fs).x, st.FrenetToCartesian(fs).y});
    }

    // 反转 Inner Lane 顺序 (使其符合顺时针驾驶方向)
    std::reverse(pd.waypoints_inner.begin(), pd.waypoints_inner.end());
    std::reverse(pd.waypoints_inner_inner_boundary.begin(), pd.waypoints_inner_inner_boundary.end());
    std::reverse(pd.waypoints_inner_outer_boundary.begin(), pd.waypoints_inner_outer_boundary.end());

    // --- Step 3: 默认激活 Outer Lane ---
    pd.waypoints = pd.waypoints_outer;
    pd.inner_boundary = pd.waypoints_outer_inner_boundary;
    pd.outer_boundary = pd.waypoints_outer_outer_boundary;

    // --- Step 4: 基于高密度点集重建 Spline ---
    // 因为现在点非常密且平滑，这里重建的 Spline 会拥有极佳的导数性质
    std::vector<double> new_sx, new_sy;
    for(const auto& p : pd.waypoints) {
        new_sx.push_back(p.x());
        new_sy.push_back(p.y());
    }
    pd.spline.init(new_sx, new_sy);

    pd.t_params.clear();
    pd.t_params = pd.spline.s;

    return pd;
}

// ... (get_spline_properties, project_to_spline_newton 等其他函数保持原样) ...
SplineProps Utils::get_spline_properties(Spline2D& spline, double t) {
    SplineProps props;
    props.pos = spline.get_pos(t);
    Eigen::Vector2d dr = spline.get_derivative(t, 1);
    Eigen::Vector2d d2r = spline.get_derivative(t, 2);
    double speed = dr.norm();
    Eigen::Vector2d T;
    if (speed < 1e-9) T = Eigen::Vector2d(1, 0);
    else T = dr / speed;
    props.tangent = T;
    props.normal_left = Eigen::Vector2d(-T.y(), T.x());
    if (speed < 1e-9) props.curvature = 0;
    else props.curvature = (dr.x() * d2r.y() - dr.y() * d2r.x()) / std::pow(speed, 3);
    return props;
}

double Utils::project_to_spline_newton(const Eigen::Vector2d& P, Spline2D& spline, double t_guess) {
    double t = t_guess;
    double t_min = spline.get_s_min();
    double t_max = spline.get_s_max();
    double length = t_max - t_min;
    if (length < 1e-6) return t_min;

    for (int i = 0; i < 10; ++i) {
        t = fmod(t - t_min, length);
        if (t < 0) t += length;
        t += t_min;

        Eigen::Vector2d r = spline.get_pos(t);
        Eigen::Vector2d dr = spline.get_derivative(t, 1);
        Eigen::Vector2d d2r = spline.get_derivative(t, 2);
        Eigen::Vector2d error = P - r;
        double f = error.dot(dr);
        double f_prime = -dr.dot(dr) + error.dot(d2r);
        if (std::abs(f) < 1e-6) break;
        if (std::abs(f_prime) < 1e-9) break; 
        t = t - f / f_prime;
    }
    t = fmod(t - t_min, length);
    if (t < 0) t += length;
    t += t_min;
    return t;
}

Eigen::Vector4d Utils::get_state_from_spline(Spline2D& spline, double s, double speed) {
    SplineProps props = get_spline_properties(spline, s);
    double yaw = atan2(props.tangent.y(), props.tangent.x());
    return Eigen::Vector4d(props.pos.x(), props.pos.y(), speed, yaw);
}

std::vector<Eigen::Vector4d> Utils::predict_obstacle_trajectory(
        double s_start, double speed, Spline2D& spline, int N, double dt) 
{
    std::vector<Eigen::Vector4d> preds;
    preds.reserve(N + 1);
    
    double s_curr = s_start;
    double s_max = spline.get_s_max();
    double s_min = spline.get_s_min();
    double length = s_max - s_min;

    for(int i = 0; i <= N; ++i) {
        preds.push_back(get_state_from_spline(spline, s_curr, speed));
        s_curr += speed * dt;
        s_curr = fmod(s_curr - s_min, length);
        if(s_curr < 0) s_curr += length;
        s_curr += s_min;
    }
    return preds;
}

// 适配 5 状态动力学
std::vector<Eigen::Vector4d> Utils::const_velo_prediction(const Eigen::Vector4d& x0, int N, double dt, double wheelbase) {
    std::vector<Eigen::Vector4d> pred;
    pred.reserve(N + 1);
    pred.push_back(x0);
    
    Vector5d x_curr;
    x_curr << x0(0), x0(1), x0(2), x0(3), 0.0; // steer=0
    Eigen::Vector2d u_zero = Eigen::Vector2d::Zero();

    for (int i = 0; i < N; ++i) {
        Vector5d x_next = Dynamics::bicycle_model(x_curr, u_zero, dt, wheelbase);
        pred.push_back(x_next.head(4));
        x_curr = x_next;
    }
    return pred;
}

Eigen::Vector2d Utils::get_vehicle_front(const Eigen::Vector2d& pos, double yaw, double wheelbase) {
    return pos + 0.5 * wheelbase * Eigen::Vector2d(cos(yaw), sin(yaw));
}

Eigen::Vector2d Utils::get_vehicle_rear(const Eigen::Vector2d& pos, double yaw, double wheelbase) {
    return pos - 0.5 * wheelbase * Eigen::Vector2d(cos(yaw), sin(yaw));
}

void Utils::get_ellipsoid_scales(double ego_radius, double obs_w, double obs_l, double d_safe, double& a, double& b) {
    a = 0.5 * obs_l + d_safe + ego_radius;
    b = 0.5 * obs_w + d_safe + ego_radius;
}

double Utils::ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const Eigen::Vector2d& center, double theta, double a, double b) {
    Eigen::Vector2d diff = pnt - center;
    Eigen::Matrix2d rot;
    rot << cos(theta), -sin(theta), sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rot.transpose() * diff;
    return 1.0 - (std::pow(pnt_std.x(), 2) / (a*a) + std::pow(pnt_std.y(), 2) / (b*b));
}

Eigen::Matrix<double, 1, 2> Utils::ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt, const Eigen::Vector2d& center, double theta, double a, double b) {
    Eigen::Vector2d diff = pnt - center;
    Eigen::Matrix2d rot;
    rot << cos(theta), -sin(theta), sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rot.transpose() * diff; 
    
    Eigen::Matrix<double, 1, 2> res_over_pnt_std;
    res_over_pnt_std << -2.0 * pnt_std.x() / (a*a), -2.0 * pnt_std.y() / (b*b);
    return res_over_pnt_std * rot.transpose();
}

void Utils::get_vehicle_front_rear_derivatives(double yaw, double wheelbase, Eigen::Matrix<double, 2, 4>& front_jac, Eigen::Matrix<double, 2, 4>& rear_jac) {
    double hw = 0.5 * wheelbase;
    front_jac.setZero();
    front_jac(0,0) = 1; front_jac(1,1) = 1; 
    front_jac(0,3) = -hw * sin(yaw);
    front_jac(1,3) = hw * cos(yaw);

    rear_jac.setZero();
    rear_jac(0,0) = 1; rear_jac(1,1) = 1;
    rear_jac(0,3) = hw * sin(yaw);
    rear_jac(1,3) = -hw * cos(yaw);
}