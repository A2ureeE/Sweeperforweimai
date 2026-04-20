"""
Periodic cubic spline for 2D path representation.

Two independent cubic splines sx(s), sy(s) form Spline2D(s) = [sx(s), sy(s)].
Supports periodic boundary conditions (closed track).

Based on CILQR_v10/src/Spline.cpp.
"""
import math
import numpy as np


class CubicSpline1D:
    """
    1D cubic spline with periodic or natural boundary conditions.

    The canonical formula for segment i over [x_i, x_{i+1}] with h = x_{i+1}-x_i:
      S(u) = M_i/6 * (h-u)^3/h + M_{i+1}/6 * u^3/h
             + (y_i - M_i*h^2/6) * (h-u)/h
             + (y_{i+1} - M_{i+1}*h^2/6) * u/h
    where u = t - x_i, and M_i is the second derivative at knot i.
    """

    def __init__(self, y: np.ndarray, periodic: bool = True):
        n = len(y)
        assert n >= 4, "Need at least 4 knots"
        self.y = np.asarray(y, dtype=float)
        self.n = n
        self.periodic = periodic
        self._t_vals = np.arange(n, dtype=float)
        self._build()

    def _build(self):
        n = self.n
        if self.periodic:
            self._build_periodic()
        else:
            self._build_natural()

    def _build_natural(self):
        n = self.n
        h = np.diff(self._t_vals)
        A = np.zeros((n, n))
        b = np.zeros(n)
        A[0, 0] = h[0] / 3.0
        A[0, 1] = h[0] / 6.0
        b[0] = (self.y[1] - self.y[0]) / h[0]
        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1] / 6.0
            A[i, i] = (h[i - 1] + h[i]) / 3.0
            A[i, i + 1] = h[i] / 6.0
            b[i] = (self.y[i + 1] - self.y[i]) / h[i] - (self.y[i] - self.y[i - 1]) / h[i - 1]
        A[-1, -1] = h[-1] / 3.0
        A[-1, -2] = h[-1] / 6.0
        b[-1] = (self.y[-1] - self.y[-2]) / h[-1]
        self.M = np.linalg.solve(A, b)
        self.h = h

    def _build_periodic(self):
        """Periodic spline: y[0] == y[-1], y[1] == y[-2]."""
        n = self.n
        # Extend by 2 points on each side for periodic boundary
        y_e = np.concatenate([self.y[-2:], self.y, self.y[:2]])
        h = np.diff(np.arange(len(y_e), dtype=float))
        N = len(y_e) - 1
        A = np.zeros((N, N))
        br = np.zeros(N)
        A[0, 0] = 2.0 * (h[0] + h[1])
        A[0, 1] = h[1]
        A[0, -1] = h[0]
        br[0] = 3.0 * ((y_e[2] - y_e[1]) / h[1] - (y_e[1] - y_e[0]) / h[0])
        for i in range(1, N - 1):
            A[i, i - 1] = h[i]
            A[i, i] = 2.0 * (h[i] + h[i + 1])
            A[i, i + 1] = h[i + 1]
            br[i] = (3.0 * ((y_e[i + 2] - y_e[i + 1]) / h[i + 1]
                            - (y_e[i + 1] - y_e[i]) / h[i]))
        A[N - 1, N - 2] = h[N - 1]
        A[N - 1, N - 1] = 2.0 * (h[N - 1] + h[0])
        A[N - 1, 0] = h[0]
        br[N - 1] = (3.0 * ((y_e[1] - y_e[0]) / h[0]
                            - (y_e[0] - y_e[-1]) / h[N - 1]))
        self.M_ext = np.linalg.solve(A, br)
        self.h_ext = h
        self.y_ext = y_e

    def _segment(self, t: float):
        """Return (y0, y1, m0, m1, h, u) for the segment containing t."""
        if self.periodic:
            t_mod = t % self.n
            if t_mod < 0:
                t_mod += self.n
            i = int(t_mod)
            u = t_mod - i
            ext_i = i + 2
            y0 = self.y_ext[ext_i]
            y1 = self.y_ext[ext_i + 1]
            m0 = self.M_ext[ext_i % len(self.M_ext)]
            m1 = self.M_ext[(ext_i + 1) % len(self.M_ext)]
            h = self.h_ext[ext_i]
        else:
            if t <= 0:
                return self.y[0], self.y[0], self.M[0], self.M[0], self.h[0], 0.0
            if t >= self.n - 1:
                return self.y[-1], self.y[-1], self.M[-1], self.M[-1], self.h[-1], 1.0
            i = int(t)
            u = t - i
            y0, y1 = self.y[i], self.y[i + 1]
            m0, m1 = self.M[i], self.M[i + 1]
            h = self.h[i]
        return y0, y1, m0, m1, h, u

    def evaluate(self, t: float) -> float:
        """Evaluate spline value at parameter t."""
        y0, y1, m0, m1, h, u = self._segment(t)
        if abs(h) < 1e-8:
            return y0
        hu = h - u
        return (m0 / 6.0 * (hu ** 3) / h
                + m1 / 6.0 * (u ** 3) / h
                + (y0 - m0 * h * h / 6.0) * hu / h
                + (y1 - m1 * h * h / 6.0) * u / h)

    def derivative(self, t: float) -> float:
        """Evaluate first derivative."""
        y0, y1, m0, m1, h, u = self._segment(t)
        if abs(h) < 1e-8:
            return 0.0
        h2 = h * h
        return (m0 / 6.0 * (-3 * (h - u) ** 2) / h
                + m1 / 6.0 * (3 * u ** 2) / h
                - (y0 - m0 * h2 / 6.0) / h
                + (y1 - m1 * h2 / 6.0) / h)


class Spline2D:
    """
    2D parametric spline: S(s) = [sx(s), sy(s)].

    Provides position, tangent, left-normal, and curvature at arc-length s.
    """

    def __init__(self, x_pts: np.ndarray, y_pts: np.ndarray, periodic: bool = True):
        self.sx = CubicSpline1D(np.asarray(x_pts, dtype=float), periodic=periodic)
        self.sy = CubicSpline1D(np.asarray(y_pts, dtype=float), periodic=periodic)
        self.n = len(x_pts)
        self.periodic = periodic
        self._s_max = self._arc_length()

    def _arc_length(self) -> float:
        """Approximate total arc length by sampling."""
        n_samp = max(200, self.n * 20)
        s = 0.0
        prev = np.array([self.sx.evaluate(0.0), self.sy.evaluate(0.0)])
        for i in range(1, int(n_samp) + 1):
            t = i * self.n / n_samp
            cur = np.array([self.sx.evaluate(t), self.sy.evaluate(t)])
            s += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            prev = cur
        return s

    def get_s_max(self) -> float:
        return self._s_max

    def _param_at_s(self, s: float) -> float:
        """Map arc-length s to parameter t (0..n)."""
        if self.periodic:
            return (s * self.n / self._s_max) % self.n
        else:
            t = s * self.n / self._s_max
            return max(0.0, min(self.n - 1e-6, t))

    def get_pos(self, s: float) -> np.ndarray:
        """World position at arc-length s."""
        t = self._param_at_s(s)
        return np.array([self.sx.evaluate(t), self.sy.evaluate(t)])

    def get_tangent(self, s: float) -> np.ndarray:
        """Unit tangent vector at arc-length s."""
        t = self._param_at_s(s)
        d = np.array([self.sx.derivative(t), self.sy.derivative(t)])
        n = np.linalg.norm(d)
        return d / n if n > 1e-6 else d

    def get_normal_left(self, s: float) -> np.ndarray:
        """Left normal (90° CCW of tangent)."""
        tx, ty = self.get_tangent(s)
        return np.array([-ty, tx])

    def get_curvature(self, s: float) -> float:
        """Curvature kappa = (x'*y'' - y'*x'') / |t|^3."""
        t = self._param_at_s(s)
        dx = self.sx.derivative(t)
        dy = self.sy.derivative(t)
        # Second derivative via finite differences
        eps = 0.02
        ddx = (self.sx.derivative(t + eps) - self.sx.derivative(t - eps)) / (2 * eps)
        ddy = (self.sy.derivative(t + eps) - self.sy.derivative(t - eps)) / (2 * eps)
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return 0.0
        return (dx * ddy - dy * ddx) / (norm ** 3)

    def project_point(self, px: float, py: float, s_guess: float = None,
                      max_iter: int = 15) -> float:
        """
        Newton-Raphson projection of (px, py) onto spline.
        Returns arc-length s of the closest point.
        Matches CILQR_v10/src/Utils.cpp::project_to_spline_newton.
        """
        # Coarse search
        if s_guess is None:
            best_s, best_d = 0.0, float('inf')
            n_search = min(100, self.n * 5)
            for i in range(n_search + 1):
                si = i * self._s_max / n_search
                p = self.get_pos(si)
                d = math.hypot(p[0] - px, p[1] - py)
                if d < best_d:
                    best_d = d
                    best_s = si
            s = best_s
        else:
            s = s_guess

        # Newton refinement
        for _ in range(max_iter):
            pos = self.get_pos(s)
            tang = self.get_tangent(s)
            r = np.array([px - pos[0], py - pos[1]])
            denom = np.dot(tang, tang)
            if denom < 1e-10:
                break
            ds = np.dot(tang, r) / denom
            s = s + ds
            if self.periodic:
                s = s % self._s_max
            else:
                s = max(0.0, min(self._s_max, s))
            if abs(ds) < 1e-4:
                break
        return s


# ---- Spline properties bundle (matches CILQR_v10/include/Utils.h SplineProps) ----
class SplineProps:
    """Bundle of spline properties at a given arc-length s."""

    def __init__(self, spline: Spline2D, s: float):
        self.pos: np.ndarray = spline.get_pos(s)
        self.tangent: np.ndarray = spline.get_tangent(s)
        self.normal_left: np.ndarray = spline.get_normal_left(s)
        self.curvature: float = spline.get_curvature(s)

    def __repr__(self):
        return (f"SplineProps(pos=[{self.pos[0]:.2f},{self.pos[1]:.2f}], "
                f"kappa={self.curvature:.4f})")
