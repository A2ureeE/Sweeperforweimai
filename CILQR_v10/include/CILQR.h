#pragma once
#include "Config.h"
#include "Utils.h"
#include <Eigen/Dense>
#include <vector>

struct Obstacle {
    std::string type;
    Eigen::Vector4d state;
    double wheelbase;
    double width;
    double length;
    double d_safe;
};

class CILQR {
public:
    Config cfg;
    Matrix5d state_weight;
    Matrix5d terminal_state_weight;
    Eigen::Matrix2d ctrl_weight;
    
    int last_start_idx = 0; 
    std::vector<double> last_iter_s_values; 

    CILQR(const Config& config);

    void solve(const Vector5d& x0, Utils& utils, PathData& path_data, 
               const std::vector<TrajectoryPoint>& ref_traj, 
               const std::vector<Obstacle>& obs_list,
               const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
               const std::vector<Vector5d>& prev_opti_x,
               const std::vector<Eigen::Vector2d>& prev_opti_u,
               std::vector<Eigen::Vector2d>& opti_u,
               std::vector<Vector5d>& opti_x);

private:
    std::vector<Eigen::Vector2d> get_nominal_control(const std::vector<Eigen::Vector2d>& prev_u);
    std::vector<Vector5d> rollout_trajectory(const Vector5d& x0, const std::vector<Eigen::Vector2d>& u);
    double angdiff(double a, double b);
    
    std::vector<SplineProps> calculate_all_projections(const std::vector<Vector5d>& x, PathData& path_data);

    Vector5d get_ref_state_at_t(double t, const std::vector<TrajectoryPoint>& ref_traj, const SplineProps& proj);

    double get_total_cost(const std::vector<Eigen::Vector2d>& u, 
                          const std::vector<Vector5d>& x,
                          const std::vector<SplineProps>& projections,
                          const std::vector<TrajectoryPoint>& ref_traj,
                          const std::vector<Obstacle>& obs_list,
                          const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
                          const std::vector<Vector5d>& prev_opti_x);

    void get_total_cost_derivatives(
        const std::vector<Eigen::Vector2d>& u, 
        const std::vector<Vector5d>& x,
        const std::vector<SplineProps>& projections,
        const std::vector<TrajectoryPoint>& ref_traj, 
        const std::vector<Obstacle>& obs_list,
        const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
        const std::vector<Vector5d>& prev_opti_x,
        std::vector<Vector5d>& l_x,
        std::vector<Matrix5d>& l_xx,
        std::vector<Eigen::Vector2d>& l_u,
        std::vector<Eigen::Matrix2d>& l_uu);

    bool backward_pass(const std::vector<Eigen::Vector2d>& u, 
                       const std::vector<Vector5d>& x,
                       double lamb,
                       const std::vector<SplineProps>& projections,
                       const std::vector<TrajectoryPoint>& ref_traj,
                       const std::vector<Obstacle>& obs_list,
                       const std::vector<std::vector<Eigen::Vector4d>>& obs_pred_list,
                       const std::vector<Vector5d>& prev_opti_x,
                       std::vector<Eigen::Vector2d>& k_gain,
                       std::vector<Matrix25d>& K_gain); // [修复] K矩阵变为 2x5 (Matrix25d)

    void forward_pass(const std::vector<Eigen::Vector2d>& u, 
                      const std::vector<Vector5d>& x,
                      const std::vector<Eigen::Vector2d>& k, 
                      const std::vector<Matrix25d>& K, // [修复] Matrix25d
                      double alpha,
                      std::vector<Eigen::Vector2d>& new_u,
                      std::vector<Vector5d>& new_x);
};