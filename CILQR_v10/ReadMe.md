CILQR High-Performance Autonomous Driving Motion Planner (5-State Model)

Introduction

This project implements a high-performance hierarchical motion planning system for autonomous driving based on C++. The system utilizes a "Coarse Planning (DP) + Fine Planning (CILQR)" dual-layer architecture, capable of handling complex dynamic traffic scenarios (e.g., multi-lane racing tracks, dynamic obstacle avoidance).

Latest Version Features (v2.0):
The core dynamic model has been upgraded from the traditional 4-state model to a 5-State Kinematic Bicycle Model.

State: $[x, y, v, \theta, \delta]$ (Steering angle $\delta$ is now a state variable)

Control: $[a, \dot{\delta}]$ (Directly controlling Steering Rate $\dot{\delta}$)

This improvement guarantees continuity of the steering angle from a physical perspective, eliminating control spikes and achieving a smooth, "experienced driver-like" driving experience.

Key Features

1. Advanced Dynamics (5-State Model)

Incorporates steering angle $\delta$ into the state vector and changes control input to steering rate $\dot{\delta}$.

Effectively limits the angular velocity of the steering wheel (Steering Rate Limit), avoiding high-frequency oscillations.

Supports RK4 (Runge-Kutta 4th Order) integration for high-precision simulation.

2. Hierarchical Architecture

Global Planner (DP Planner):

Constructs a sampling graph in the Frenet ($s-l$) frame.

Uses Dynamic Programming to search the convex space.

Considers spatiotemporal predictions of dynamic obstacles to generate a collision-free coarse reference path (Path-Velocity Profile).

Local Planner (CILQR Planner):

Nonlinear Model Predictive Control (NMPC) based on Constrained Iterative LQR (iLQR).

Uses Exponential Barrier Functions to handle hard constraints (road boundaries, obstacle avoidance).

Implements Analytical Derivatives calculation, significantly boosting solution speed.

3. Robust Numerical Optimization

Regularization: Dynamic Levenberg-Marquardt regularization to handle non-positive definite Hessian matrices.

Line Search: Adaptive step size adjustment ensuring monotonic decrease of the cost function.

Warm Start: Accelerates iteration convergence by utilizing the previous frame's solution and projection cache.

4. Real-time Visualization

Real-time simulation interface based on matplotlib-cpp.

HUD support: Real-time speed, steering angle, lateral error, and obstacle safety ellipses.

Dependencies

The project is developed in a Linux environment and requires the following libraries:

C++ Compiler: Supports C++14 or higher (g++ / clang++).

CMake: Version >= 3.10.

Eigen 3: Core linear algebra library.

Python 3 & Matplotlib: Used for visualization.

Install Python dev package: sudo apt install python3-dev

Install Matplotlib: pip3 install matplotlib

Build & Run

1. Build

mkdir build
cd build
cmake ..
make -j4


2. Run

./CILQR_Sim


After running, a Matplotlib window will pop up, displaying the real-time planning and motion of the vehicle on the track.

System Architecture

graph TD
    A[Perception & Prediction] --> B(DP Coarse Planning<br>Dynamic Programming);
    B -->|Coarse Trajectory & Speed Profile| C{CILQR Optimizer<br>Trajectory Optimization};
    D[Static Map<br>Spline/Reference Line] --> B;
    D --> C;
    E[Vehicle State<br>x, y, v, yaw, steer] --> B;
    E --> C;
    C -->|Optimal Control u| F[Vehicle Dynamics Simulation<br>RK4 Integration];
    F --> E;


Performance Metrics

Based on simulation tests comparable to real-vehicle industrial PC levels (analyzed from log.txt data):

Metric

Description

Typical Value

Evaluation

Computation Time

Total planning time per frame

~26 ms

Extremely Fast (Supports >30Hz)

Steering Smoothness

Steering Rate

< 12 deg/s

Smooth (Far below EPS limits)

Control Precision

Speed/Path tracking error

Good Convergence

Stable

Safety

Obstacle avoidance distance

> Safety Margin

Collision Free

Configuration

All core parameters are defined in src/Config.cpp and do not require header file modifications:

cfg.mpc.w_steer_rate: Controls steering smoothness. Larger values make the steering slower/steadier.

cfg.vehicle.max_steer_rate: Physical constraint, limiting maximum steering speed (rad/s).

cfg.track.num_smooth_points: Track discretization density. Increasing this value eliminates sharp corners on road boundaries.

cfg.dp.w_collision: Obstacle avoidance weight in the DP stage.

File Structure

main.cpp: Main loop, orchestrating prediction, DP, CILQR, and simulation updates.

CILQR.cpp/.h: Core solver, containing Forward/Backward Pass.

DPPlanner.cpp/.h: Dynamic planner, generating initial solutions (Warm Start).

Dynamics.cpp/.h: 5-state bicycle model and its Jacobian derivation.

Constraints.cpp/.h: Implementation of Barrier Functions for road and obstacle constraints.

StateTransformer.cpp/.h: Transformation between Cartesian and Frenet coordinate systems.

Utils.cpp/.h: Geometry calculations, spline interpolation, collision detection tools.

Visualizer.cpp/.h: Visualization module.

FAQ

Q: Why does the ego vehicle sometimes deviate from the centerline?
A: This is to avoid dynamic obstacles. The DP layer plans a detour path, and CILQR smoothly tracks this path.

Q: How to tune the driving style?
A: Modify the weight matrix R in Config.cpp. Increasing w_acc makes acceleration gentler; increasing w_steer_rate makes steering lazier.

Author: [VictorLiu]
Date: 2025-12-31