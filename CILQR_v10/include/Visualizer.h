#pragma once
#include "Config.h"
#include "Utils.h" 
#include "CILQR.h"
#include <Eigen/Dense>
#include <vector>
#include <string>

// 前向声明
struct Obstacle;

// [新增] 用于存储分析数据的结构体
struct AnalysisData {
    std::vector<double> time;
    std::vector<double> speed;       // 实际速度
    std::vector<double> ref_speed;   // 参考速度
    std::vector<double> accel;       // 控制量：加速度
    std::vector<double> steer;       // 状态量：转向角
    std::vector<double> steer_rate;  // 控制量：转向速率
    std::vector<double> lat_error;   // 横向误差
    std::vector<double> yaw;         // 航向角
};

class Visualizer {
public:
    Visualizer(const Config& config);

    // 实时绘制 (保持不变)
    void draw(const Eigen::Vector4d& ego_state, 
              const Eigen::Vector2d& current_u,
              double ref_speed,
              const std::vector<Obstacle>& obs_list,
              const std::vector<Eigen::Vector4d>& plan_x,
              const std::vector<Eigen::Vector4d>& dp_trajectory,
              const PathData& path_data,
              const std::vector<Eigen::Vector4d>& history_ego);

    // [新增] 仿真结束后的详细分析绘图
    void plot_analysis(const AnalysisData& data);

private:
    Config cfg;
    void draw_vehicle(const Eigen::Vector4d& state, std::string color, double length, double width);
    void draw_obstacle_ellipse(const Obstacle& obs);
};