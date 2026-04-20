#include "Spline.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>

// --- CubicSpline (三次样条插值) 实现 ---
// 用于一维数据的平滑插值，保证二阶导数连续

CubicSpline::CubicSpline() : nx(0) {}

// 初始化周期性样条 (闭合曲线)
void CubicSpline::init_periodic(const std::vector<double>& x_in, const std::vector<double>& y_in) {
    x = x_in;
    y = y_in;
    nx = x.size();

    // 计算步长 h
    int n = nx - 1;
    std::vector<double> h(n);
    for (int i = 0; i < n; ++i) {
        h[i] = x[i+1] - x[i];
    }

    // 构建线性方程组 Ax = B 来求解二阶导数 M
    // 周期性边界条件意味着起点和终点的导数连续
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(n);

    for (int i = 0; i < n; ++i) {
        int prev = (i - 1 + n) % n;
        int next = (i + 1) % n;
        // 填充三对角矩阵 (考虑周期性，角落有元素)
        if (i == 0) {
            A(0, 0) = 2.0 * (h[n-1] + h[0]);
            A(0, 1) = h[0];
            A(0, n-1) = h[n-1];// 周期项
        } else if (i == n - 1) {
            A(n-1, n-2) = h[n-2];
            A(n-1, n-1) = 2.0 * (h[n-2] + h[n-1]);
            A(n-1, 0) = h[n-1];// 周期项
        } else {
            A(i, i-1) = h[i-1];
            A(i, i)   = 2.0 * (h[i-1] + h[i]);
            A(i, i+1) = h[i];
        }

        // 计算 B 向量 (差分项)
        double term1 = (y[next] - y[i]) / h[i];
        double term2 = 0.0;
        if (i == 0) term2 = (y[0] - y[n-1]) / h[n-1];
        else term2 = (y[i] - y[i-1]) / h[i-1];
        
        B(i) = 6.0 * (term1 - term2);
    }

    // 求解线性方程组得到 M (二阶导数值)
    Eigen::VectorXd M = A.colPivHouseholderQr().solve(B);

    // 计算三次多项式系数 a, b, c, d
    // y(x) = a + b*dt + c*dt^2 + d*dt^3
    a.resize(n); b.resize(n); c.resize(n); d.resize(n);
    for (int i = 0; i < n; ++i) {
        int next = (i + 1) % n;
        a[i] = y[i];
        b[i] = (y[next] - y[i]) / h[i] - (2.0 * h[i] * M(i) + h[i] * M(next)) / 6.0;
        c[i] = M(i) / 2.0;
        d[i] = (M(next) - M(i)) / (6.0 * h[i]);
    }
}

// 计算样条函数值或其导数
double CubicSpline::calc(double t, int order) {
    // NaN 检查，防止非法输入崩溃
    if (std::isnan(t)) return 0.0;

    double range = x.back() - x.front();
    // 处理周期性映射，将 t 映射到 [x_start, x_end]
    double t_mod = fmod(t - x.front(), range);
    if (t_mod < 0) t_mod += range;
    t_mod += x.front();

    // 二分查找找到 t 所在的区间 idx
    auto it = std::upper_bound(x.begin(), x.end(), t_mod);
    int idx = std::distance(x.begin(), it) - 1;
    if (idx < 0) idx = 0;
    if (idx >= (int)a.size()) idx = a.size() - 1;

    double dt = t_mod - x[idx];
    
    // 根据阶数返回 0阶(位置), 1阶(速度), 2阶(加速度)
    if (order == 0) {
        return a[idx] + b[idx] * dt + c[idx] * dt * dt + d[idx] * dt * dt * dt;
    } else if (order == 1) {
        return b[idx] + 2.0 * c[idx] * dt + 3.0 * d[idx] * dt * dt;
    } else if (order == 2) {
        return 2.0 * c[idx] + 6.0 * d[idx] * dt;
    }
    return 0.0;
}

// --- Spline2D 实现 ---
// 结合两个 CubicSpline (x(s), y(s)) 来表示二维平面曲线
// 参数 s 为累计弧长
Spline2D::Spline2D() : total_length(0.0) {}

void Spline2D::init(const std::vector<double>& x, const std::vector<double>& y) {
    s.clear();
    s.push_back(0.0);
    // 计算累计弧长 s
    for (size_t i = 0; i < x.size() - 1; ++i) {
        double dx = x[i+1] - x[i];
        double dy = y[i+1] - y[i];
        s.push_back(s.back() + std::sqrt(dx*dx + dy*dy));
    }
    total_length = s.back();
    // 分别对 x 和 y 关于 s 进行样条插值
    sx.init_periodic(s, x);
    sy.init_periodic(s, y);
}

// 获取二维位置 (x, y)
Eigen::Vector2d Spline2D::get_pos(double t) {
    return Eigen::Vector2d(sx.calc(t, 0), sy.calc(t, 0));
}

// 获取二维导数 (dx/ds, dy/ds) 等
Eigen::Vector2d Spline2D::get_derivative(double t, int order) {
    return Eigen::Vector2d(sx.calc(t, order), sy.calc(t, order));
}

double Spline2D::get_s_min() { return s.front(); }
double Spline2D::get_s_max() { return s.back(); }