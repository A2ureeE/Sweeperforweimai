#include "Config.h"

Config Config::getConfig() {
    Config cfg;
    
    // --- MPC 基本参数 ---
    cfg.mpc.N = 40; 
    cfg.mpc.dt = 0.1;
    cfg.mpc.nx = 5; // 状态: [x, y, v, yaw, steer]
    cfg.mpc.nu = 2; // 控制: [acc, steer_rate]

    // --- 权重 ---
    cfg.mpc.w_pos = 8.0;
    cfg.mpc.w_vel = 2.0;
    cfg.mpc.w_yaw_process = 1.0;
    cfg.mpc.w_steer = 5.0; 
    
    // R 矩阵
    cfg.mpc.w_acc = 1.0;
    cfg.mpc.w_steer_rate = 80.0; 
    
    // 终端代价
    cfg.mpc.w_pos_terminal = 20.0;
    cfg.mpc.w_vel_terminal = 5.0;
    cfg.mpc.w_yaw_terminal = 5.0;
    cfg.mpc.w_steer_terminal = 5.0; 
    
    cfg.mpc.w_consistency = 3.0;

    // 障碍函数
    cfg.mpc.exp_q1 = 20.5;
    cfg.mpc.exp_q2 = 1.5;

    // --- iLQR 迭代 ---
    cfg.iteration.max_iter = 15;
    cfg.iteration.init_lamb = 5.0;
    cfg.iteration.lamb_decay = 0.7;
    cfg.iteration.lamb_amplify = 2.0;
    cfg.iteration.max_lamb = 10000.0;
    cfg.iteration.alpha_options = {1.0, 0.5, 0.25, 0.125, 0.0625};
    cfg.iteration.tol = 0.01;

    // --- 车辆 ---
    cfg.vehicle.wheelbase = 2.94;
    cfg.vehicle.width = 1.6;
    cfg.vehicle.length = 3.5;
    cfg.vehicle.velo_max = 20.0;
    cfg.vehicle.velo_min = 0.0;
    cfg.vehicle.a_max = 3.0;
    cfg.vehicle.a_min = -4.0;
    cfg.vehicle.stl_lim = 0.6; 
    cfg.vehicle.max_steer_rate = 0.5; 

    // --- 道路 ---
    cfg.road.width = 12.0;
    cfg.road.velo_ref = 30/3.6; 
    cfg.mpc.road_exp_q1 = 100.0;
    cfg.mpc.road_exp_q2 = 10.0;
    cfg.mpc.road_safe_margin = 1.2;

    // --- ego初始条件 ---
    cfg.initial_condition = {10.0, 40.0, 0.0, -M_PI/2.0, 0.0};

    // --- 赛道生成参数 ---
    cfg.track.step_curve = 0.1 * M_PI; // 原 0.1 -> 0.02 (每 3.6度 一个点)
    cfg.track.step_line = 3.0;          // 原 4.0 -> 1.0 (每 1米 一个点)
    cfg.track.num_smooth_points = 2000; // 增加最终生成的平滑点数
    
    cfg.track.segments = {
        {"arc",  {30, 30, 20, M_PI,      1.5*M_PI}},
        {"line", {30, 10, 80, 10}},
        {"arc",  {80, 30, 20, -0.5*M_PI, 0}},
        {"line", {100, 30, 100, 40}},
        {"arc",  {80, 40, 20, 0,       0.5*M_PI}},
        {"line", {80, 60, 30, 60}},
        {"arc",  {30, 40, 20, 0.5*M_PI,  M_PI}},
        {"line", {10, 40, 10, 30}}
    };

    // --- DP 参数 ---
    cfg.dp.rows = 29;             
    cfg.dp.cols = 20;              
    cfg.dp.sample_s_step = 3.0;  
    
    cfg.dp.w_smooth_dl = 70.0;
    cfg.dp.w_smooth_ddl = 100.0;
    cfg.dp.w_smooth_dddl = 1000.0;
    cfg.dp.w_ref = 15.0;           
    cfg.dp.w_collision = 1000000.0;

    return cfg;
}