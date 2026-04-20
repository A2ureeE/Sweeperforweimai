#pragma once
#include <vector>
#include <string>
#include <cmath>
#include "Utils.h" // 包含 TrackConfig 定义

// 定义各类参数结构体
struct MPCConfig {
    int N;
    double dt;
    int nx; // [修改] 现在是 5
    int nu; // [修改] 保持 2，但含义变为 [acc, steer_rate]
    
    // 权重参数
    double w_pos;
    double w_vel;
    double w_yaw_process;
    double w_steer;       // [新增] 转向角状态误差权重 (原 w_stl 移到这里)
    
    double w_acc;         // 加速度控制权重
    double w_steer_rate;  // [新增] 转向速率控制权重 (替代原 w_stl)
    
    double w_pos_terminal;
    double w_vel_terminal;
    double w_yaw_terminal;
    double w_steer_terminal; // [新增] 终端转向角权重
    
    double w_consistency;
    
    // Barrier Function 参数
    double exp_q1;
    double exp_q2;
    double road_exp_q1;
    double road_exp_q2;
    double road_safe_margin;
};

struct IterationConfig {
    int max_iter;
    double init_lamb;
    double lamb_decay;
    double lamb_amplify;
    double max_lamb;
    std::vector<double> alpha_options;
    double tol;
};

struct VehicleConfig {
    double wheelbase;
    double width;
    double length;
    double velo_max;
    double velo_min;
    double a_max;
    double a_min;
    double stl_lim;       // [语义变化] 现在是状态约束
    double max_steer_rate;// [新增] 最大转向速率 (rad/s)，控制约束
};

struct RoadConfig {
    double width;
    double velo_ref;
};

struct InitialCondition {
    double x; double y; double v; double yaw; double steer; // [新增] steer
};

// [新增] DP Planner 配置参数
struct DPConfig {
    int rows;              // 横向采样点数
    int cols;              // 纵向(S方向)层数
    double sample_s_step;  // 纵向采样步长 (m)
    
    // 代价权重
    double w_smooth_dl;    // 平滑性权重 (一阶导)
    double w_smooth_ddl;   // 平滑性权重 (二阶导)
    double w_smooth_dddl;  // 平滑性权重 (三阶导)
    double w_ref;          // 偏离参考线权重 (L的代价)
    double w_collision;    // 碰撞代价权重
};

class Config {
public:
    MPCConfig mpc;
    IterationConfig iteration;
    VehicleConfig vehicle;
    RoadConfig road;
    InitialCondition initial_condition;
    TrackConfig track;
    DPConfig dp; 

    // 静态方法获取单例配置或默认配置
    static Config getConfig();
};