# sweeper_control package
# MPC CILQR solver and controller nodes

# Re-export all public symbols so users can do:
#   from sweeper_control import CILQRSolver, Spline2D, rk4
from .bicycle_dynamics import rk4, linearize, rollout, angdiff
from .spline import Spline2D, SplineProps, CubicSpline1D
from .cilqr_solver import (
    CILQRSolver, MPCConfig, IterationConfig,
    VehicleConfig, RoadConfig, Obstacle, TrajPoint,
)
