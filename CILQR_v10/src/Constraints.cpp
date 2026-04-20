#include "Constraints.h"
#include <cmath>
#include <algorithm>
#include <iostream>

/**
 * @brief 指数型 Barrier 函数
 * * 用于将不等式约束 (c <= 0) 转化为代价函数中的惩罚项。
 * 形式类似于 Sigmoid 函数的反向。
 * * 公式: b(c) = q1 / (1 + exp(-q2 * c))
 * * @param c 约束违反程度 (Constraint Violation). 
 * c > 0 表示违反约束 (Penalty 激增); 
 * c < 0 表示满足约束 (Penalty 趋近 0).
 * @param q1 惩罚上限系数 (Scaling factor)
 * @param q2 惩罚陡峭程度 (Steepness). 越大越接近硬约束，但也越难优化。
 */
double Constraints::exp_barrier(double c, double q1, double q2) {
    return q1 / (1.0 + std::exp(-q2 * c));
}

/**
 * @brief 计算 Barrier 函数对状态 x (或控制 u) 的一阶和二阶导数
 * * 利用链式法则: 
 * d(Barrier)/dx = d(Barrier)/dc * dc/dx
 * * @param c 当前约束值
 * @param c_dot 约束 c 对变量 x 的导数 (Jacobian)
 * @param b_dot [输出] Barrier 对 x 的梯度
 * @param b_ddot [输出] Barrier 对 x 的 Hessian
 */
void Constraints::exp_barrier_derivative_and_Hessian(double c, const Eigen::MatrixXd& c_dot, double q1, double q2, Eigen::MatrixXd& b_dot, Eigen::MatrixXd& b_ddot) {
    double b = exp_barrier(c, q1, q2);
    // Sigmoid 导数性质: b' = q2 * b * (1 - b/q1)
    // 这是标量 b 对标量 c 的导数
    double db_dc = q2 * b * (1.0 - b / q1);
    // 链式法则一阶导: Gradient = (db/dc) * (dc/dx)
    b_dot = db_dc * c_dot;
    // 链式法则二阶导: Hessian approx = (db/dc)^2 * (dc/dx)^T * (dc/dx)
    // 注意：这里忽略了 d2b/dc2 项，采用 Gauss-Newton 风格的近似，
    // 保证 Hessian 正定性，有利于优化收敛。
    b_ddot = (db_dc * db_dc) * (c_dot.transpose() * c_dot);
}

// 简单的边界约束辅助函数：upper -> val - bound <= 0
double Constraints::get_bound_constr(double var, double bound, const std::string& type) {
    if (type == "upper") return var - bound;
    else return bound - var;               
}

// --- 道路廊道约束 ---
// 计算车辆距离道路边界的距离。
// c = |dist_to_center| - (road_width/2 - margin)
// 如果 c > 0，说明这就超出安全边界了
double Constraints::get_road_corridor_constr(const Vector5d& x, const SplineProps& proj, double road_width, double safe_margin) {
    Eigen::Vector2d pos = x.head(2);
    // 投影点到车位置的向量
    Eigen::Vector2d diff = pos - proj.pos;
    // 在 Frenet 法线方向上的投影长度 (即横向偏差 l)
    double e = diff.dot(proj.normal_left);
    double half_allow = std::max(road_width / 2.0 - safe_margin, 0.0);
    return std::abs(e) - half_allow;
}

// 道路约束及其导数
void Constraints::get_road_corridor_constr_and_deriv(const Vector5d& x, const SplineProps& proj, double road_width, double safe_margin, double& c, Eigen::MatrixXd& c_dot) {
    Eigen::Vector2d pos = x.head(2);
    double e = (pos - proj.pos).dot(proj.normal_left);
    double half_allow = std::max(road_width / 2.0 - safe_margin, 0.0);
    // 约束值
    c = std::abs(e) - half_allow;
    // 符号函数，用于处理绝对值导数
    double sgn = 0.0;
    if (e > 1e-6) sgn = 1.0;
    else if (e < -1e-6) sgn = -1.0;

    // c_dot 是 1x5 矩阵: [dx, dy, dv, dyaw, dsteer]
    // 只有 x, y 对道路横向误差有影响
    // d(|e|)/dx = sgn * n_x
    // d(|e|)/dy = sgn * n_y
    c_dot = Eigen::MatrixXd::Zero(1, 5);
    c_dot(0, 0) = sgn * proj.normal_left(0);
    c_dot(0, 1) = sgn * proj.normal_left(1);
}

// --- 障碍物避让约束 ---
// 计算车辆与障碍物的安全余量
// 使用椭圆近似：Margin = 1 - (x/a)^2 - (y/b)^2
// 在椭圆内(碰撞)，(x/a)^2 < 1 -> return > 0.
// 在椭圆外(安全)，(x/a)^2 > 1 -> return < 0.
// 所以：正值 = 碰撞/危险。Barrier 惩罚正值。逻辑正确。
void Constraints::get_obstacle_avoidance_constr(
    const Vector5d& ego_state, 
    const Eigen::Vector4d& obs_state, 
    double ego_wheelbase, double ego_width, 
    double obs_width, double obs_length, double d_safe,
    double& front_margin, double& rear_margin) 
{
    double a, b;
    // 获取膨胀后的椭圆半轴长
    Utils::get_ellipsoid_scales(ego_width / 2.0, obs_width, obs_length, d_safe, a, b);

    // 计算自车前后轴中心的物理坐标
    // 5状态向量中: index 3 是 yaw
    Eigen::Vector2d ego_front = Utils::get_vehicle_front(ego_state.head(2), ego_state(3), ego_wheelbase);
    Eigen::Vector2d ego_rear = Utils::get_vehicle_rear(ego_state.head(2), ego_state(3), ego_wheelbase);
    
    // 计算两个检测点的 Margin
    front_margin = Utils::ellipsoid_safety_margin(ego_front, obs_state.head(2), obs_state(3), a, b);
    rear_margin  = Utils::ellipsoid_safety_margin(ego_rear,  obs_state.head(2), obs_state(3), a, b);
}

// 障碍物约束导数 (Chain Rule Heavy Lifting)
void Constraints::get_obstacle_avoidance_constr_derivatives(
    const Vector5d& ego_state, 
    const Eigen::Vector4d& obs_state, 
    double ego_wheelbase, double ego_width, 
    double obs_width, double obs_length, double d_safe,
    Eigen::MatrixXd& front_deriv, 
    Eigen::MatrixXd& rear_deriv)
{
    double a, b;
    Utils::get_ellipsoid_scales(ego_width / 2.0, obs_width, obs_length, d_safe, a, b);
    
    Eigen::Vector2d ego_front = Utils::get_vehicle_front(ego_state.head(2), ego_state(3), ego_wheelbase);
    Eigen::Vector2d ego_rear = Utils::get_vehicle_rear(ego_state.head(2), ego_state(3), ego_wheelbase);
    
    // 1. Margin 对 检测点坐标 (px, py) 的导数 (1x2)
    Eigen::Matrix<double, 1, 2> front_margin_over_pnt = Utils::ellipsoid_safety_margin_derivatives(
        ego_front, obs_state.head(2), obs_state(3), a, b);
        
    Eigen::Matrix<double, 1, 2> rear_margin_over_pnt = Utils::ellipsoid_safety_margin_derivatives(
        ego_rear, obs_state.head(2), obs_state(3), a, b);
        
    // 2. 检测点坐标 对 车辆状态 (x, y, v, yaw) 的导数 (2x4)
    // Utils::get_vehicle_front_rear_derivatives 返回 2x4 (x, y, v, yaw)
    Eigen::Matrix<double, 2, 4> front_pnt_over_state_4d, rear_pnt_over_state_4d;
    Utils::get_vehicle_front_rear_derivatives(ego_state(3), ego_wheelbase, front_pnt_over_state_4d, rear_pnt_over_state_4d);
    
    // 3. 扩展到 5D 状态: [x, y, v, yaw, steer]
    // steer 对当前时刻的车体前后点坐标无直接几何影响 (它影响的是下一时刻的状态)
    // 所以第5列 (steer) 的导数为 0
    Eigen::Matrix<double, 2, 5> front_pnt_over_state_5d = Eigen::Matrix<double, 2, 5>::Zero();
    front_pnt_over_state_5d.block<2, 4>(0, 0) = front_pnt_over_state_4d;

    Eigen::Matrix<double, 2, 5> rear_pnt_over_state_5d = Eigen::Matrix<double, 2, 5>::Zero();
    rear_pnt_over_state_5d.block<2, 4>(0, 0) = rear_pnt_over_state_4d;

    // 4. 链式法则最终乘积 (1x2 * 2x5 = 1x5)
    front_deriv = front_margin_over_pnt * front_pnt_over_state_5d;
    rear_deriv  = rear_margin_over_pnt * rear_pnt_over_state_5d;
}