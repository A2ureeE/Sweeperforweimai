"""
5-state bicycle model + analytical Jacobians.

State  x = [x, y, v, yaw, steer]  (x, y in m; v in m/s; yaw, steer in rad)
Control u = [acc, steer_rate]       (acc in m/s^2; steer_rate in rad/s)

Jacobians A (5x5) and B (5x2) are derived analytically:
  x_dot = [v*cos(yaw), v*sin(yaw), acc, v*tan(steer)/L, steer_rate]

Derivation (see CILQR_v10/src/Dynamics.cpp):
  A[0,2] = cos(yaw)*dt         A[0,3] = -v*sin(yaw)*dt
  A[1,2] = sin(yaw)*dt         A[1,3] =  v*cos(yaw)*dt
  A[3,2] = tan(steer)/L*dt     A[3,4] = v*sec^2(steer)/L*dt
  B[2,0] = dt   (acc -> v)
  B[4,1] = dt   (steer_rate -> steer)
All other entries = 0 (identity for remaining diagonal terms).
"""
import math
import numpy as np


def continuous_ode(x: np.ndarray, u: np.ndarray, L: float) -> np.ndarray:
    """5-state continuous bicycle ODE. x=(5,), u=(2,)."""
    v, yaw, steer = x[2], x[3], x[4]
    acc, steer_rate = u[0], u[1]
    x_dot = np.empty(5)
    x_dot[0] = v * math.cos(yaw)
    x_dot[1] = v * math.sin(yaw)
    x_dot[2] = acc
    x_dot[3] = v * math.tan(steer) / L
    x_dot[4] = steer_rate
    return x_dot


def rk4(x: np.ndarray, u: np.ndarray, L: float, dt: float) -> np.ndarray:
    """4th-order Runge-Kutta integration. x=(5,), u=(2,). Returns x_next=(5,)."""
    k1 = continuous_ode(x, u, L)
    k2 = continuous_ode(x + 0.5 * dt * k1, u, L)
    k3 = continuous_ode(x + 0.5 * dt * k2, u, L)
    k4 = continuous_ode(x + dt * k3, u, L)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def linearize(x_traj, u_traj, L: float, dt: float, N: int):
    """
    Compute A[i] (5x5) and B[i] (5x2) for i=0..N-1 using analytical formulas.

    State order: [x, y, v, yaw, steer]
    Control order: [acc, steer_rate]
    """
    A_list = []
    B_list = []
    for i in range(N):
        v = x_traj[i][2]
        yaw = x_traj[i][3]
        steer = x_traj[i][4]
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        ts = math.tan(steer)
        sec_sq = 1.0 + ts * ts

        A = np.eye(5)
        A[0, 2] = cy * dt
        A[0, 3] = -v * sy * dt
        A[1, 2] = sy * dt
        A[1, 3] = v * cy * dt
        A[3, 2] = ts / L * dt
        A[3, 4] = v * sec_sq / L * dt

        B = np.zeros((5, 2))
        B[2, 0] = dt   # acc -> v
        B[4, 1] = dt   # steer_rate -> steer

        A_list.append(A)
        B_list.append(B)
    return A_list, B_list


def rollout(x0: np.ndarray, u_seq, L: float, dt: float) -> list:
    """Forward integrate N steps. u_seq: list of N (2,) arrays or (N,2) array."""
    N = len(u_seq)
    x_traj = [np.array(x0, dtype=float)]
    for i in range(N):
        u = np.asarray(u_seq[i])
        x_next = rk4(x_traj[-1], u, L, dt)
        x_traj.append(x_next)
    return x_traj


def angdiff(a: float, b: float) -> float:
    """Signed angular difference a - b, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))
