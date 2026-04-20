"""
Constrained Iterative LQR (CILQR) solver.

Ported from CILQR_v10/src/CILQR.cpp.

Algorithm (per solve() call):
  1. Warm start: shift previous control sequence by one step.
  2. Forward rollout: integrate dynamics to get nominal trajectory.
  3. Iterative loop:
       a. Compute A, B Jacobians (Dynamics.linearize).
       b. Compute cost derivatives l_x, l_xx, l_u, l_uu (get_total_cost_derivatives).
       c. Backward pass: Riccati recursion -> gains k[i] (2,) and K[i] (2,5).
       d. Forward pass (line search over alpha) -> new u, x.
       e. Levenberg-Marquardt lambda update on fail.
  4. Return optimized u[0] (acc, steer_rate) for the first step.

Key differences from naive iLQR:
  - Exponential barrier functions for soft inequality constraints.
  - Consistency cost: penalize deviation from previous solution.
  - Warm start: use shifted previous solution.
  - Spline projection caching for road-constraint evaluation.
"""
import math
import numpy as np
from dataclasses import dataclass, field

from .bicycle_dynamics import rk4, linearize, rollout, angdiff
from .spline import Spline2D, SplineProps
from .spline import Spline2D as _S2D, SplineProps as _SP
from .bicycle_dynamics import rk4 as _rk4, angdiff as _ad


# -----------------------------------------------------------------------
# Config classes
# -----------------------------------------------------------------------
@dataclass
class MPCConfig:
    N: int = 20
    dt: float = 0.1
    nx: int = 5
    nu: int = 2
    # State weights [x, y, v, yaw, steer]
    w_pos: float = 8.0
    w_vel: float = 2.0
    w_yaw: float = 1.0
    w_steer: float = 5.0
    # Control weights [acc, steer_rate]
    w_acc: float = 1.0
    w_steer_rate: float = 80.0
    # Terminal state weights
    w_pos_term: float = 20.0
    w_vel_term: float = 5.0
    w_yaw_term: float = 5.0
    w_steer_term: float = 5.0
    # Consistency weight
    w_consistency: float = 3.0
    # Barrier function parameters
    exp_q1: float = 1.0
    exp_q2: float = 10.0
    road_exp_q1: float = 5.0
    road_exp_q2: float = 5.0
    road_safe_margin: float = 0.5


@dataclass
class IterationConfig:
    max_iter: int = 15
    init_lamb: float = 5.0
    lamb_decay: float = 0.7
    lamb_amplify: float = 2.0
    max_lamb: float = 1e4
    alpha_options: list = field(
        default_factory=lambda: [1.0, 0.5, 0.25, 0.125, 0.0625])
    tol: float = 0.01


@dataclass
class VehicleConfig:
    wheelbase: float = 1.05
    width: float = 1.05
    length: float = 1.52
    velo_max: float = 1.3
    velo_min: float = -0.3
    a_max: float = 0.8
    a_min: float = -1.0
    stl_lim: float = 0.6          # max steering angle (rad)
    max_steer_rate: float = 0.5   # max steering rate (rad/s)


@dataclass
class RoadConfig:
    width: float = 12.0
    velo_ref: float = 30.0 / 3.6  # 30 km/h in m/s


# -----------------------------------------------------------------------
# Obstacle (matches CILQR_v10/include/Utils.h Obstacle struct)
# -----------------------------------------------------------------------
@dataclass
class Obstacle:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    width: float = 0.6
    length: float = 0.6
    d_safe: float = 0.5


# -----------------------------------------------------------------------
# Reference trajectory point (matches TrajectoryPoint in CILQR_v10)
# -----------------------------------------------------------------------
@dataclass
class TrajPoint:
    x: float
    y: float
    v: float
    theta: float
    kappa: float
    t: float
    s: float = 0.0
    l: float = 0.0


# -----------------------------------------------------------------------
# Exponential barrier helpers
# -----------------------------------------------------------------------
def exp_barrier(c: float, q1: float, q2: float) -> float:
    """
    Exponential barrier: b(c) = q1 / (1 + exp(-q2*c))

    c > 0 -> constraint violated -> large penalty
    c < 0 -> constraint satisfied -> near-zero penalty
    """
    if c > 50.0:
        return q1
    if c < -50.0:
        return 0.0
    return q1 / (1.0 + math.exp(-q2 * c))


def exp_barrier_grad_hess(c: float, c_dot: np.ndarray, q1: float, q2: float):
    """
    Compute gradient and Gauss-Newton Hessian of exp_barrier.

    b(c) = q1 / (1+exp(-q2*c))
    db/dc = q2 * b * (1 - b/q1)          [sigmoid derivative]
    dJ/dx = (db/dc) * (dc/dx)           [chain rule]
    d2J/dx2 ≈ (db/dc)^2 * (dc/dx)^T * (dc/dx)  [Gauss-Newton approx]

    Args:
        c: scalar constraint value
        c_dot: (n,) gradient of c w.r.t. variables [should be row-vector shaped]
    Returns:
        grad: (n,) gradient vector
        hess: (n, n) approximate Hessian
    """
    b = exp_barrier(c, q1, q2)
    db_dc = q2 * b * (1.0 - b / q1)
    grad = db_dc * c_dot
    hess = (db_dc ** 2) * np.outer(c_dot, c_dot)
    return grad, hess


def bound_constr(var: float, bound: float, kind: str) -> float:
    """
    Bound constraint: upper -> var - bound <= 0
                       lower -> bound - var <= 0
    """
    return var - bound if kind == "upper" else bound - var


# -----------------------------------------------------------------------
# Road corridor constraint
# -----------------------------------------------------------------------
def road_corridor_constr(x: np.ndarray, proj: SplineProps,
                         road_w: float, margin: float) -> float:
    """
    c = |lateral_error| - (road_w/2 - margin)
    positive -> outside road -> constraint violated.
    """
    diff = x[:2] - proj.pos          # (2,)
    e = float(np.dot(diff, proj.normal_left))  # signed lateral deviation
    half = max(road_w / 2.0 - margin, 0.0)
    return abs(e) - half


def road_constr_grad(x: np.ndarray, proj: SplineProps) -> np.ndarray:
    """
    Gradient of road corridor constraint w.r.t. 5D state.
    Only x, y affect lateral error -> gradient entries 0,1.
    """
    diff = x[:2] - proj.pos
    e = float(np.dot(diff, proj.normal_left))
    sgn = 1.0 if e > 1e-6 else (-1.0 if e < -1e-6 else 0.0)
    grad = np.zeros(5)
    grad[0] = sgn * proj.normal_left[0]
    grad[1] = sgn * proj.normal_left[1]
    return grad


# -----------------------------------------------------------------------
# Obstacle avoidance (ellipse safety margin)
# -----------------------------------------------------------------------
def ellipsoid_scales(ego_r: float, obs_w: float, obs_l: float,
                     d_safe: float) -> tuple:
    """
    Compute ellipse semi-axes for rotated bounding ellipse.
    a = 0.5 * (w + l) + r_ego + d_safe
    b = min(w, l) / max(w, l) * a
    """
    a = 0.5 * (obs_w + obs_l) + ego_r + d_safe
    b = min(obs_w, obs_l) / max(obs_w, obs_l, 1e-6) * a
    b = max(b, 0.15)
    return a, b


def vehicle_front_rear(pos_xy: np.ndarray, yaw: float, L: float
                       ) -> tuple:
    """Return (front, rear) 2D positions.  State is rear-axle midpoint."""
    c, s = math.cos(yaw), math.sin(yaw)
    half_L = L / 2.0
    rear = pos_xy - half_L * np.array([c, s])
    front = pos_xy + half_L * np.array([c, s])
    return front, rear


def ellipsoid_margin(pt: np.ndarray, obs_xy: np.ndarray,
                     obs_yaw: float, a: float, b: float) -> float:
    """
    Ellipse safety margin at point pt:
      margin = 1 - (dx/a)^2 - (dy/b)^2  in rotated frame
    margin > 0 = safe, margin < 0 = collision.
    """
    dx = pt[0] - obs_xy[0]
    dy = pt[1] - obs_xy[1]
    c, s = math.cos(-obs_yaw), math.sin(-obs_yaw)
    xr = c * dx - s * dy
    yr = s * dx + c * dy
    return 1.0 - (xr / a) ** 2 - (yr / b) ** 2


def ellipsoid_margin_grad(pt: np.ndarray, obs_xy: np.ndarray,
                          obs_yaw: float, a: float, b: float) -> np.ndarray:
    """Gradient of ellipse margin w.r.t. pt (2D)."""
    dx = pt[0] - obs_xy[0]
    dy = pt[1] - obs_xy[1]
    c, s = math.cos(-obs_yaw), math.sin(-obs_yaw)
    xr = c * dx - s * dy
    yr = s * dx + c * dy
    # d(dx_r)/d(px) = c, d(dx_r)/d(py) = -s
    # d(dy_r)/d(px) = s, d(dy_r)/d(py) = c
    grad = np.zeros(2)
    denom = a * a * b * b
    grad[0] = -(2.0 * xr / a / a) * c - (2.0 * yr / b / b) * s
    grad[1] = -(-2.0 * xr / a / a) * s - (2.0 * yr / b / b) * c
    return grad


# -----------------------------------------------------------------------
# CILQR Solver
# -----------------------------------------------------------------------
class CILQRSolver:
    """
    Constrained iLQR solver for 5-state bicycle model.

    State: x = [x, y, v, yaw, steer]
    Control: u = [acc, steer_rate]
    """

    def __init__(self, mpc_cfg: MPCConfig,
                 iter_cfg: IterationConfig,
                 veh_cfg: VehicleConfig,
                 road_cfg: RoadConfig):
        self.mpc = mpc_cfg
        self.it = iter_cfg
        self.veh = veh_cfg
        self.road = road_cfg

        N = mpc_cfg.N
        self._Q = np.diag([mpc_cfg.w_pos, mpc_cfg.w_pos,
                            mpc_cfg.w_vel, mpc_cfg.w_yaw, mpc_cfg.w_steer])
        self._Q_term = np.diag([mpc_cfg.w_pos_term, mpc_cfg.w_pos_term,
                                  mpc_cfg.w_vel_term, mpc_cfg.w_yaw_term,
                                  mpc_cfg.w_steer_term])
        self._R = np.diag([mpc_cfg.w_acc, mpc_cfg.w_steer_rate])

        # Warm-start cache
        self._last_s_values: list = []
        self._last_start_idx: int = 0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _ref_state(self, t: float, ref_traj: list) -> np.ndarray:
        """Interpolate reference state from DP trajectory or use road defaults."""
        ref = np.zeros(5)
        ref[2] = self.road.velo_ref

        if ref_traj:
            # Binary search for t
            lo, hi = 0, len(ref_traj) - 1
            while lo < hi:
                mid = (lo + hi + 1) // 2
                if ref_traj[mid].t <= t:
                    lo = mid
                else:
                    hi = mid - 1

            if lo == 0:
                p = ref_traj[0]
            elif lo >= len(ref_traj) - 1:
                p = ref_traj[-1]
            else:
                p0, p1 = ref_traj[lo], ref_traj[lo + 1]
                dt = p1.t - p0.t
                if dt < 1e-5:
                    p = p0
                else:
                    alpha = (t - p0.t) / dt
                    ref[0] = p0.x + alpha * (p1.x - p0.x)
                    ref[1] = p0.y + alpha * (p1.y - p0.y)
                    ref[2] = p0.v + alpha * (p1.v - p0.v)
                    da = angdiff(p1.theta, p0.theta)
                    ref[3] = p0.theta + alpha * da
                    ref[4] = math.atan(self.veh.wheelbase *
                                       (p0.kappa + alpha * (p1.kappa - p0.kappa)))
                    return ref

            ref[0] = p.x
            ref[1] = p.y
            ref[2] = p.v
            ref[3] = p.theta
            ref[4] = math.atan(self.veh.wheelbase * p.kappa)

        return ref

    def _get_proj(self, x: np.ndarray, spline: Spline2D,
                   path_pts: list) -> SplineProps:
        """Project state onto spline and return SplineProps."""
        px, py = x[0], x[1]
        if self._last_s_values:
            s_guess = self._last_s_values[0] if self._last_s_values else None
        else:
            s_guess = None
        s = spline.project_point(px, py, s_guess)
        self._last_s_values.append(s)
        return SplineProps(spline, s)

    def _warm_control(self, prev_u: list) -> list:
        """Shift previous control sequence by one step."""
        N = self.mpc.N
        u = [np.zeros(2) for _ in range(N)]
        if prev_u and len(prev_u) == N:
            for i in range(N - 1):
                u[i] = prev_u[i + 1].copy()
            u[N - 1] = prev_u[N - 1].copy()
        return u

    def _forward_rollout(self, x0: np.ndarray, u: list) -> list:
        """Integrate dynamics from x0 using control sequence u."""
        return rollout(x0, u, self.veh.wheelbase, self.mpc.dt)

    # ------------------------------------------------------------------
    # Cost derivatives
    # ------------------------------------------------------------------
    def _cost_grad_hess(self, u: list, x: list,
                        projs: list,
                        ref_traj: list,
                        prev_x: list,
                        obs_list: list) -> tuple:
        """
        Compute l_x[i] (5,), l_xx[i] (5,5), l_u[i] (2,), l_uu[i] (2,2)
        for all steps i=0..N (x) and i=0..N-1 (u).
        """
        N = self.mpc.N
        l_x = [np.zeros(5) for _ in range(N + 1)]
        l_xx = [np.zeros((5, 5)) for _ in range(N + 1)]
        l_u = [np.zeros(2) for _ in range(N)]
        l_uu = [np.zeros((2, 2)) for _ in range(N)]

        # Stage cost derivatives
        for i in range(N):
            t = i * self.mpc.dt
            ref = self._ref_state(t, ref_traj)
            dx = x[i] - ref
            dx[3] = angdiff(x[i][3], ref[3])

            # Tracking cost: (x-ref)^T Q (x-ref)
            l_x[i] += 2.0 * self._Q @ dx
            l_xx[i] += 2.0 * self._Q

            # Control cost: u^T R u
            l_u[i] += 2.0 * self._R @ u[i]
            l_uu[i] += 2.0 * self._R

            # Consistency cost: penalize deviation from previous solution
            if prev_x and i < len(prev_x):
                dev = x[i] - prev_x[i]
                l_x[i] += 2.0 * self.mpc.w_consistency * dev
                l_xx[i] += 2.0 * self.mpc.w_consistency * np.eye(5)

            # ---- Barrier: acceleration limits ----
            acc = u[i][0]
            sr = u[i][1]
            c_u = np.array([1.0, 0.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(acc, self.veh.a_max, "upper"),
                c_u, self.mpc.exp_q1, self.mpc.exp_q2)
            l_u[i] += g; l_uu[i] += H

            c_u = np.array([-1.0, 0.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(acc, self.veh.a_min, "lower"),
                c_u, self.mpc.exp_q1, self.mpc.exp_q2)
            l_u[i] += g; l_uu[i] += H

            # ---- Barrier: steer rate limits ----
            c_u = np.array([0.0, 1.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(sr, self.veh.max_steer_rate, "upper"),
                c_u, self.mpc.exp_q1, self.mpc.exp_q2)
            l_u[i] += g; l_uu[i] += H

            c_u = np.array([0.0, -1.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(sr, -self.veh.max_steer_rate, "lower"),
                c_u, self.mpc.exp_q1, self.mpc.exp_q2)
            l_u[i] += g; l_uu[i] += H

        # Terminal cost
        t_term = N * self.mpc.dt
        ref_term = self._ref_state(t_term, ref_traj)
        dx_t = x[N] - ref_term
        dx_t[3] = angdiff(x[N][3], ref_term[3])
        l_x[N] += 2.0 * self._Q_term @ dx_t
        l_xx[N] += 2.0 * self._Q_term

        # ---- State barriers (k = 1..N) ----
        for k in range(1, N + 1):
            xk = x[k]
            proj = projs[k]

            # Velocity bounds
            c_v = np.array([0.0, 0.0, 1.0, 0.0, 0.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(xk[2], self.veh.velo_max, "upper"),
                c_v, self.mpc.exp_q1, self.mpc.exp_q2)
            l_x[k] += g; l_xx[k] += H

            c_v = np.array([0.0, 0.0, -1.0, 0.0, 0.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(xk[2], self.veh.velo_min, "lower"),
                c_v, self.mpc.exp_q1, self.mpc.exp_q2)
            l_x[k] += g; l_xx[k] += H

            # Steering angle bounds
            c_s = np.array([0.0, 0.0, 0.0, 0.0, 1.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(xk[4], self.veh.stl_lim, "upper"),
                c_s, self.mpc.exp_q1, self.mpc.exp_q2)
            l_x[k] += g; l_xx[k] += H

            c_s = np.array([0.0, 0.0, 0.0, 0.0, -1.0])
            g, H = exp_barrier_grad_hess(
                bound_constr(xk[4], -self.veh.stl_lim, "lower"),
                c_s, self.mpc.exp_q1, self.mpc.exp_q2)
            l_x[k] += g; l_xx[k] += H

            # Road corridor barrier
            c_road = road_corridor_constr(xk, proj,
                                           self.road.width,
                                           self.mpc.road_safe_margin)
            c_grad = road_constr_grad(xk, proj)
            g, H = exp_barrier_grad_hess(
                c_road, c_grad,
                self.mpc.road_exp_q1, self.mpc.road_exp_q2)
            l_x[k] += g; l_xx[k] += H

            # Obstacle barriers (front + rear axle check)
            for obs in obs_list:
                a, b = ellipsoid_scales(self.veh.width / 2.0,
                                        obs.width, obs.length, obs.d_safe)
                front, rear = vehicle_front_rear(xk[:2], xk[3],
                                                  self.veh.wheelbase)
                obs_yaw = math.atan2(obs.vy, obs.vx + 1e-6)
                for pt, sign in [(front, 1.0), (rear, 1.0)]:
                    m = ellipsoid_margin(pt, np.array([obs.x, obs.y]),
                                         obs_yaw, a, b)
                    m_grad = ellipsoid_margin_grad(
                        pt, np.array([obs.x, obs.y]), obs_yaw, a, b)
                    # Extend to 5D gradient (only x,y matter)
                    c_obs = np.zeros(5)
                    c_obs[:2] = m_grad
                    g, H = exp_barrier_grad_hess(
                        m, c_obs, self.mpc.exp_q1, self.mpc.exp_q2)
                    l_x[k] += g; l_xx[k] += H

        return l_x, l_xx, l_u, l_uu

    # ------------------------------------------------------------------
    # Backward pass
    # ------------------------------------------------------------------
    def _backward_pass(self, u: list, x: list,
                       projs: list,
                       ref_traj: list,
                       prev_x: list,
                       obs_list: list,
                       lamb: float
                       ) -> tuple:
        """
        Riccati recursion. Returns (k_gain, K_gain, success).

        k_gain[i]  : (2,) feedforward
        K_gain[i]  : (2,5) feedback
        """
        N = self.mpc.N
        A_list, B_list = linearize(x, u, self.veh.wheelbase,
                                     self.mpc.dt, N)
        l_x, l_xx, l_u, l_uu = self._cost_grad_hess(
            u, x, projs, ref_traj, prev_x, obs_list)

        # Terminal Value Function
        Vx = l_x[N].copy()
        Vxx = l_xx[N].copy()

        k_gain = [np.zeros(2) for _ in range(N)]
        K_gain = [np.zeros((2, 5)) for _ in range(N)]

        regI = lamb * np.eye(5)
        ok = True

        for i in range(N - 1, -1, -1):
            A = A_list[i]
            B = B_list[i]

            # Q = l + A^T V A
            Qx = l_x[i] + A.T @ Vx
            Qu = l_u[i] + B.T @ Vx

            Qxx = l_xx[i] + A.T @ Vxx @ A
            Quu = l_uu[i] + B.T @ Vxx @ B
            Qux = B.T @ Vxx @ A

            # Levenberg-Marquardt regularization
            Quu_reg = Quu + B.T @ regI @ B

            # Cholesky decomposition
            try:
                L = np.linalg.cholesky(Quu_reg)
                lower = True
            except np.linalg.LinAlgError:
                Quu_reg += 1e-4 * np.eye(2)
                try:
                    L = np.linalg.cholesky(Quu_reg)
                    lower = True
                except np.linalg.LinAlgError:
                    ok = False
                    break

            # Solve L L^T k = -Qu  =>  k = -L^{-T} L^{-1} Qu
            k_tilde = np.linalg.solve(L.T, np.linalg.solve(L, -Qu))
            k_gain[i] = k_tilde

            # K = -L^{-T} L^{-1} Qux
            K_tilde = np.linalg.solve(L.T, np.linalg.solve(L, Qux))
            K_gain[i] = -K_tilde

            # Value function update (second-order approximation)
            Vx = Qx + K_tilde.T @ Quu @ k_tilde + K_tilde.T @ Qu + Qux.T @ k_tilde
            Vxx = Qxx + K_tilde.T @ Quu @ K_tilde + K_tilde.T @ Qux + Qux.T @ K_tilde

        return k_gain, K_gain, ok

    # ------------------------------------------------------------------
    # Forward pass
    # ------------------------------------------------------------------
    def _forward_pass(self, u_nom: list, x_nom: list,
                      k: list, K: list,
                      alpha: float) -> tuple:
        """
        Apply control law: u_new = u_nom + alpha*k + K*(x_new - x_nom).
        Returns (new_u, new_x).
        """
        N = self.mpc.N
        new_u = [np.zeros(2) for _ in range(N)]
        new_x = [np.array(x_nom[0])]
        for i in range(N):
            delta_x = new_x[-1] - x_nom[i]
            u_raw = u_nom[i] + alpha * k[i] + K[i] @ delta_x
            # Hard clip to physical limits
            u_clip = np.array([
                max(min(u_raw[0], self.veh.a_max), self.veh.a_min),
                max(min(u_raw[1], self.veh.max_steer_rate),
                    -self.veh.max_steer_rate),
            ])
            new_u[i] = u_clip
            x_next = rk4(new_x[-1], u_clip, self.veh.wheelbase, self.mpc.dt)
            new_x.append(x_next)
        return new_u, new_x

    # ------------------------------------------------------------------
    # Main solver entry
    # ------------------------------------------------------------------
    def solve(self, x0: np.ndarray,
              spline: Spline2D,
              path_pts: list,
              ref_traj: list,
              prev_opt_u: list,
              prev_opt_x: list,
              obs_list: list) -> np.ndarray:
        """
        Run CILQR for initial state x0.

        Args:
            x0: (5,) initial state
            spline: Spline2D reference path
            path_pts: list of (N+1) waypoints for spline construction
            ref_traj: list of TrajPoint from DP planner (can be empty)
            prev_opt_u: previous optimized control sequence (N, 2)
            prev_opt_x: previous optimized state sequence (N+1, 5)
            obs_list: list of Obstacle objects

        Returns:
            exec_u: (2,) first-step optimized control [acc, steer_rate]
        """
        if np.any(np.isnan(x0)) or np.any(np.isinf(x0)):
            return np.array([0.0, 0.0])

        self._last_s_values = []
        N = self.mpc.N

        # Warm start
        u = self._warm_control(prev_opt_u)
        x = self._forward_rollout(x0, u)

        # Projections cache
        projs = [SplineProps(spline,
                             spline.project_point(x[i][0], x[i][1]))
                 for i in range(N + 1)]

        # Initial cost
        prev_x = prev_opt_x if prev_opt_x else x
        l_x, l_xx, l_u, l_uu = self._cost_grad_hess(
            u, x, projs, ref_traj, prev_x, obs_list)
        J = sum(
            float((x[i] - self._ref_state(i * self.mpc.dt, ref_traj)).T
                  @ self._Q
                  @ (x[i] - self._ref_state(i * self.mpc.dt, ref_traj)))
            + float(u[i].T @ self._R @ u[i])
            for i in range(N)
        )

        lamb = self.it.init_lamb

        for itr in range(self.it.max_iter):
            # Backward pass
            k_gain, K_gain, bp_ok = self._backward_pass(
                u, x, projs, ref_traj, prev_x, obs_list, lamb)

            if not bp_ok:
                lamb *= self.it.lamb_amplify
                if lamb > self.it.max_lamb:
                    break
                continue

            # Forward pass with line search
            fp_ok = False
            new_J = J
            for alpha in self.it.alpha_options:
                new_u, new_x = self._forward_pass(u, x, k_gain, K_gain, alpha)
                # Check for NaN
                if any(np.any(np.isnan(xi)) for xi in new_x):
                    continue
                # Truncate s-cache to N+1
                proj_cache_slice = self._last_s_values[:N + 1]
                l_x2, l_xx2, l_u2, l_uu2 = self._cost_grad_hess(
                    new_u, new_x, projs, ref_traj, prev_x, obs_list)
                tJ = sum(
                    float((new_x[i] - self._ref_state(i * self.mpc.dt, ref_traj)).T
                          @ self._Q
                          @ (new_x[i] - self._ref_state(i * self.mpc.dt, ref_traj)))
                    + float(new_u[i].T @ self._R @ new_u[i])
                    for i in range(N)
                )
                if tJ < J:
                    u = new_u
                    x = new_x
                    J = tJ
                    fp_ok = True
                    lamb *= self.it.lamb_decay
                    break

            if not fp_ok:
                lamb *= self.it.lamb_amplify
                if lamb > self.it.max_lamb:
                    break

            # Convergence check
            if abs(J - new_J) < self.it.tol:
                break

        return u[0].copy()
