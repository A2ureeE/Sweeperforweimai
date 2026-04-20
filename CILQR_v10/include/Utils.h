#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "Spline.h"
// #include "Config.h" // 避免循环依赖，Config在cpp中包含

// [新增] 5状态模型所需的 Eigen 类型定义
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 5, 2> Matrix52d; // 用于 B 矩阵 (5x2)
typedef Eigen::Matrix<double, 2, 5> Matrix25d; // [修复] 用于 K 矩阵 (2x5)

// 轨迹点结构体，用于 DP Planner 输出
struct TrajectoryPoint {
    double x;
    double y;
    double v;
    double theta;
    double kappa;
    double t; // 相对时间
    double s; // Frenet S
    double l; // Frenet L
};

// PathData 结构体：包含多车道数据
struct PathData {
    Spline2D spline; // [核心] 这是 MPC 计算横向误差的数学基准
    std::vector<double> t_params; 

    // 1. Center Line (L=0)
    std::vector<Eigen::Vector2d> waypoints_center;
    std::vector<Eigen::Vector2d> waypoints_center_inner_boundary; 
    std::vector<Eigen::Vector2d> waypoints_center_outer_boundary; 

    // 2. Inner Lane (CW)
    std::vector<Eigen::Vector2d> waypoints_inner;
    std::vector<Eigen::Vector2d> waypoints_inner_inner_boundary; 
    std::vector<Eigen::Vector2d> waypoints_inner_outer_boundary; 

    // 3. Outer Lane (CCW)
    std::vector<Eigen::Vector2d> waypoints_outer;
    std::vector<Eigen::Vector2d> waypoints_outer_inner_boundary; 
    std::vector<Eigen::Vector2d> waypoints_outer_outer_boundary; 
    
    // 当前激活路径
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<Eigen::Vector2d> inner_boundary;
    std::vector<Eigen::Vector2d> outer_boundary;
};

struct SplineProps {
    Eigen::Vector2d pos;
    Eigen::Vector2d tangent;
    Eigen::Vector2d normal_left;
    double curvature;
};

struct TrackSegment {
    std::string type; 
    std::vector<double> params; 
};

struct TrackConfig {
    double step_curve;
    double step_line;
    int num_smooth_points;
    std::vector<TrackSegment> segments;
};

class Utils {
public:
    static PathData generate_racetrack_data(const TrackConfig& track_def, double road_width);
    
    static SplineProps get_spline_properties(Spline2D& spline, double t);
    
    static double project_to_spline_newton(const Eigen::Vector2d& P, Spline2D& spline, double t_guess);
    
    static std::vector<Eigen::Vector4d> predict_obstacle_trajectory(
        double s_start, double speed, Spline2D& spline, int N, double dt);

    static Eigen::Vector4d get_state_from_spline(Spline2D& spline, double s, double speed);

    static std::vector<Eigen::Vector4d> const_velo_prediction(const Eigen::Vector4d& x0, int N, double dt, double wheelbase);

    static Eigen::Vector2d get_vehicle_front(const Eigen::Vector2d& pos, double yaw, double wheelbase);
    static Eigen::Vector2d get_vehicle_rear(const Eigen::Vector2d& pos, double yaw, double wheelbase);
    
    static void get_ellipsoid_scales(double ego_radius, double obs_w, double obs_l, double d_safe, double& a, double& b);
    
    static double ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const Eigen::Vector2d& center, double theta, double a, double b);
    
    static Eigen::Matrix<double, 1, 2> ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt, const Eigen::Vector2d& center, double theta, double a, double b);
    
    static void get_vehicle_front_rear_derivatives(double yaw, double wheelbase, Eigen::Matrix<double, 2, 4>& front_jac, Eigen::Matrix<double, 2, 4>& rear_jac);
};