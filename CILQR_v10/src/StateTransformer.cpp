#include "StateTransformer.h"
#include "Utils.h"
#include <cmath>
#include <iostream>
#include <iomanip>

StateTransformer::StateTransformer(Spline2D& spline) : ref_spline(spline) {}

// 将角度归一化到 [-PI, PI]
double StateTransformer::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 计算参考线在 s 处的曲率 kappa
double StateTransformer::get_kappa(double s) {
    Eigen::Vector2d d1 = ref_spline.get_derivative(s, 1);
    Eigen::Vector2d d2 = ref_spline.get_derivative(s, 2);
    double norm_d1 = d1.norm();
    if (norm_d1 < 1e-6) return 0.0;
    // k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
    return (d1.x() * d2.y() - d1.y() * d2.x()) / std::pow(norm_d1, 3);
}

// 计算曲率对弧长 s 的导数 (dk/ds)
double StateTransformer::get_kappa_dot(double s) {
    double ds = 0.1;
    double k_plus = get_kappa(s + ds);
    double k_minus = get_kappa(s - ds);
    return (k_plus - k_minus) / (2.0 * ds);
}

// 调试函数
void StateTransformer::DebugProjection(const Eigen::Vector2d& p) {
    std::cout << "[StateTransformer] Debugging Projection for P(" << p.x() << ", " << p.y() << ")" << std::endl;
    
    double s_max = ref_spline.get_s_max();
    double min_d = 1e9;
    double best_s_guess = 0;
    
    for (double ss = 0; ss < s_max; ss += 1.0) {
        Eigen::Vector2d pt = ref_spline.get_pos(ss);
        double d = (pt - p).norm();
        if (d < min_d) { min_d = d; best_s_guess = ss; }
    }
    std::cout << "  > Coarse Search: Nearest s_guess = " << best_s_guess << ", dist = " << min_d << std::endl;

    double final_s = Utils::project_to_spline_newton(p, ref_spline, best_s_guess);
    SplineProps props = Utils::get_spline_properties(ref_spline, final_s);
    
    Eigen::Vector2d r_vec = p - props.pos;
    double l = r_vec.dot(props.normal_left);
    
    std::cout << "  > Newton Refine: Final s = " << final_s << std::endl;
    std::cout << "  > Ref Point on Spline: (" << props.pos.x() << ", " << props.pos.y() << ")" << std::endl;
    std::cout << "  > Normal Vector: (" << props.normal_left.x() << ", " << props.normal_left.y() << ")" << std::endl;
    std::cout << "  > Calculated L = " << l << std::endl;
    std::cout << "------------------------------------------------" << std::endl;
}

// --- 笛卡尔坐标 (x,y,v,theta) -> Frenet 坐标 (s, l, l', l'', s_dot, ...) ---
FrenetState StateTransformer::CartesianToFrenet(const CartesianState& cs) {
    FrenetState fs;
    Eigen::Vector2d p(cs.x, cs.y);

    double s_max = ref_spline.get_s_max();
    double t_guess = 0;
    double min_d = 1e9;
    
    // 1. 粗搜索：遍历所有采样点找最近点作为初值
    for (double ss = 0; ss < s_max; ss += 5.0) {
        Eigen::Vector2d pt = ref_spline.get_pos(ss);
        double d = (pt - p).norm();
        if (d < min_d) { min_d = d; t_guess = ss; }
    }
    // 2. 牛顿法精修：找到准确的投影点 s
    fs.s = Utils::project_to_spline_newton(p, ref_spline, t_guess);
    
    // 获取参考点属性
    SplineProps props = Utils::get_spline_properties(ref_spline, fs.s);
    double ref_theta = atan2(props.tangent.y(), props.tangent.x());
    double ref_kappa = props.curvature;
    double ref_kappa_dot = get_kappa_dot(fs.s);

    // 计算横向偏差 l
    Eigen::Vector2d r_vec = p - props.pos;
    fs.l = r_vec.dot(props.normal_left); 

    // 辅助变量
    double delta_theta = normalize_angle(cs.theta - ref_theta);
    double one_minus_kappa_l = 1.0 - ref_kappa * fs.l;
    double tan_delta = tan(delta_theta);
    double cos_delta = cos(delta_theta);

    // 计算 l' = dl/ds = (1 - k*l) * tan(delta_theta)
    fs.l_prime = one_minus_kappa_l * tan_delta;
    
    // 计算 l'' = d(l')/ds
    double d_delta_theta_ds = cs.kappa * one_minus_kappa_l / cos_delta - ref_kappa;
    fs.l_double_prime = -(ref_kappa_dot * fs.l + ref_kappa * fs.l_prime) * tan_delta + 
                        (one_minus_kappa_l / (cos_delta * cos_delta)) * d_delta_theta_ds;

    if (std::abs(one_minus_kappa_l) < 1e-4) one_minus_kappa_l = 1e-4;
    
    fs.s_dot = cs.v * cos_delta / one_minus_kappa_l;
    fs.l_dot = cs.v * sin(delta_theta);
    fs.s_ddot = cs.a; // 简化假设：s方向加速度近似等于车辆纵向加速度
    fs.l_ddot = 0;    // 简化

    return fs;
}

// --- Frenet 坐标 -> 笛卡尔坐标 ---
CartesianState StateTransformer::FrenetToCartesian(const FrenetState& fs) {
    CartesianState cs;

    // 处理 s 的循环
    double s_norm = fs.s;
    double L = ref_spline.total_length;
    if (L > 1.0) { 
        if (s_norm < 0) s_norm += L;
        if (s_norm > L) s_norm = fmod(s_norm, L);
    }

    SplineProps props = Utils::get_spline_properties(ref_spline, s_norm);
    double ref_theta = atan2(props.tangent.y(), props.tangent.x());
    double ref_kappa = props.curvature;
    
    // 计算笛卡尔坐标 (x, y) = r(s) + l * n(s)
    cs.x = props.pos.x() + fs.l * props.normal_left.x();
    cs.y = props.pos.y() + fs.l * props.normal_left.y();

    // 反解航向角 theta
    double one_minus_kappa_l = 1.0 - ref_kappa * fs.l;
    double delta_theta = atan2(fs.l_prime, one_minus_kappa_l);
    cs.theta = normalize_angle(ref_theta + delta_theta);

    double cos_delta = cos(delta_theta);
    if (std::abs(cos_delta) < 1e-4) cos_delta = 1e-4;
    
    // 反解速度 v
    cs.v = fs.s_dot * one_minus_kappa_l / cos_delta;

    // 反解曲率 (复杂公式，仅用于完整性，通常用于生成路径时不需要)
    double tan_delta = tan(delta_theta);
    double ref_kappa_dot = get_kappa_dot(fs.s);
    double term1 = ref_kappa_dot * fs.l + ref_kappa * fs.l_prime;
    double d_delta_theta_ds = (fs.l_double_prime + term1 * tan_delta) * (cos_delta * cos_delta) / one_minus_kappa_l;
    
    cs.kappa = (d_delta_theta_ds + ref_kappa) * cos_delta / one_minus_kappa_l;
    cs.a = fs.s_ddot;

    return cs;
}