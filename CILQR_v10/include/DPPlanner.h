#pragma once
#include <vector>
#include <Eigen/Dense>
#include "Config.h"
#include "StateTransformer.h"
#include "Utils.h" 
#include "CILQR.h"

class DPPlanner {
public:
    DPPlanner(const Config& cfg, StateTransformer& st);

    // 核心规划接口
    std::vector<Eigen::Vector4d> Plan(
        const Eigen::Vector4d& start_state_xy, 
        const std::vector<Obstacle>& obs_list, 
        const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list, 
        double road_left_bound,
        double road_right_bound
    );

    const std::vector<TrajectoryPoint>& GetDetailedTrajectory() const { return final_traj_; }

    std::vector<Eigen::Vector2d> GetDPPathPoints() const;

private:
    Config cfg_;
    StateTransformer& st_;
    
    std::vector<Eigen::Vector2d> dp_path_sl_;
    std::vector<TrajectoryPoint> final_traj_;

    void QuinticPolynomial(
        double start_s, double start_l, double start_dl, double start_ddl,
        double end_s, double end_l, double end_dl, double end_ddl,
        std::vector<double>& coeffs);

    // [更新] 适配新版 CPP：增加 start_time 和 end_time 参数
    double CalculateLinkCost(
        const Eigen::Vector3d& start_sl_state, 
        const Eigen::Vector3d& end_sl_state,   
        double start_s, double end_s,
        double start_time, double end_time, // 新增参数
        const std::vector<Obstacle>& obs_list,
        const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
        double road_left_bound,
        double road_right_bound
    );

    // [更新] 适配新版 CPP：增加 time 参数
    double CalcObstacleCost(double s, double l, double time, 
                            const std::vector<Obstacle>& obs_list,
                            const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list);

    void GenerateSpeedProfile(
        const std::vector<Eigen::Vector2d>& path_sl, 
        double start_v, 
        double target_v,
        std::vector<TrajectoryPoint>& out_traj);
};