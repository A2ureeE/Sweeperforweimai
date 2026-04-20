#pragma once
#include "Spline.h"
#include <Eigen/Dense>

// Frenet 坐标系状态结构体
struct FrenetState {
    double s;              // 沿参考线的弧长
    double l;              // 横向偏移 (左正右负)
    double s_dot;          // s 对时间的导数 (纵向速度)
    double l_dot;          // l 对时间的导数 (横向速度)
    double s_ddot;         // 纵向加速度
    double l_ddot;         // 横向加速度
    double l_prime;        // l 对 s 的一阶导 (dl/ds)
    double l_double_prime; // l 对 s 的二阶导 (d2l/ds2)
};

// 笛卡尔坐标系状态结构体
struct CartesianState {
    double x;
    double y;
    double v;      // 线速度
    double theta;  // 航向角
    double kappa;  // 曲率
    double a;      // 加速度
};

class StateTransformer {
public:
    // 构造函数传入参考线的样条曲线
    StateTransformer(Spline2D& ref_spline);

    // 将笛卡尔状态转换为 Frenet 状态
    FrenetState CartesianToFrenet(const CartesianState& cs);
    
    // 将 Frenet 状态转换为笛卡尔状态 (用于生成 Offset 轨迹)
    CartesianState FrenetToCartesian(const FrenetState& fs);

    // 调试用：打印投影信息
    void DebugProjection(const Eigen::Vector2d& p);

    // 角度归一化到 [-PI, PI]
    double normalize_angle(double angle);

private:
    Spline2D& ref_spline; // 引用主参考线
    
    // 获取参考线在 s 处的曲率
    double get_kappa(double s);
    // 获取参考线在 s 处的曲率变化率
    double get_kappa_dot(double s);
};