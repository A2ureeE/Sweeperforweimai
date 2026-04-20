#pragma once
#include <vector>
#include <Eigen/Dense>

class CubicSpline {
public:
    std::vector<double> x; // 节点
    std::vector<double> y; // 值
    int nx;
    std::vector<double> a, b, c, d; // 系数

    CubicSpline();
    
    // 初始化闭合样条
    void init_periodic(const std::vector<double>& x_in, const std::vector<double>& y_in);
    
    // 计算值或导数 (order: 0=pos, 1=vel, 2=acc)
    double calc(double t, int order = 0);
};

class Spline2D {
public:
    CubicSpline sx;
    CubicSpline sy;
    std::vector<double> s; // 累计距离
    double total_length;

    Spline2D();
    
    // 初始化 2D 样条
    void init(const std::vector<double>& x, const std::vector<double>& y);
    
    Eigen::Vector2d get_pos(double t);
    Eigen::Vector2d get_derivative(double t, int order);
    
    double get_s_min();
    double get_s_max();
};