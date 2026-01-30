import numpy as np
from typing import Tuple



class KalmanPosVelWithGPSBias2D:
    """
    2D KF that fuses:
      - high-rate velocity measurements (vx, vy) with white noise
      - low-rate GPS position measurements (px, py) that exhibit slow wandering drift

    State x = [px, py, vx, vy, bx, by]^T
      p: true position
      v: true velocity
      b: GPS bias (wander), modeled as AR(1): b_k = phi * b_{k-1} + w_b
    GPS measurement: z_gps = p + b + noise
    Vel measurement: z_vel = v + noise
    """

    def __init__(
        self,
        dt: float,
        sigma_acc: float = 1.0,      # (m/s^2) process noise driving velocity (maneuvers)
        sigma_bias: float = 0.02,    # (m) per-step bias process noise scale (wander rate)
        tau_bias: float = 120.0,     # (s) bias correlation time (b is slow if tau is large)
        sigma_vel_meas: float = 0.2, # (m/s) velocity measurement std
        sigma_gps_meas: float = 2.0, # (m) GPS measurement std (white part)
        # x0: np.ndarray | None = None,
        # P0: np.ndarray | None = None,
    ):
        self.dt = float(dt)

        # AR(1) coefficient for bias (Gauss?Markov)
        self.phi = float(np.exp(-self.dt / float(tau_bias)))

        # State dimension
        self.n = 6

        # State and covariance
        self.x = np.zeros((self.n, 1)) #if x0 is None else np.asarray(x0, dtype=float).reshape(self.n, 1)
        self.P = np.eye(self.n) * 10.0 #if P0 is None else np.asarray(P0, dtype=float).reshape(self.n, self.n)

        # Build constant matrices
        self.F = self._make_F(self.dt, self.phi)

        # Process noise covariance Q:
        # - Acceleration noise affects position & velocity (white accel -> CV model)
        # - Bias noise affects bx, by
        self.Q = self._make_Q(self.dt, sigma_acc, sigma_bias)

        # Measurement models
        self.H_vel = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
        ], dtype=float)

        self.R_vel = (sigma_vel_meas ** 2) * np.eye(2)

        self.H_gps = np.array([
            [1, 0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0, 1],
        ], dtype=float)

        self.R_gps = (sigma_gps_meas ** 2) * np.eye(2)

        self.I = np.eye(self.n)

    @staticmethod
    def _make_F(dt: float, phi: float) -> np.ndarray:
        F = np.eye(6, dtype=float)
        # position integrates velocity
        F[0, 2] = dt
        F[1, 3] = dt
        # bias AR(1)
        F[4, 4] = phi
        F[5, 5] = phi
        return F

    @staticmethod
    def _make_Q(dt: float, sigma_acc: float, sigma_bias: float) -> np.ndarray:
        """
        Q for a constant-velocity model driven by white acceleration noise.
        Per axis (x or y) with state [p, v]:
          Q_axis = sigma_acc^2 * [[dt^4/4, dt^3/2],
                                 [dt^3/2, dt^2  ]]
        Bias modeled as random innovation per step with variance sigma_bias^2 (on bx, by).
        """
        sa2 = float(sigma_acc) ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        Q = np.zeros((6, 6), dtype=float)

        # x-axis (px, vx)
        Q[0, 0] = sa2 * (dt4 / 4.0)
        Q[0, 2] = sa2 * (dt3 / 2.0)
        Q[2, 0] = sa2 * (dt3 / 2.0)
        Q[2, 2] = sa2 * (dt2)

        # y-axis (py, vy)
        Q[1, 1] = sa2 * (dt4 / 4.0)
        Q[1, 3] = sa2 * (dt3 / 2.0)
        Q[3, 1] = sa2 * (dt3 / 2.0)
        Q[3, 3] = sa2 * (dt2)

        # bias noise (bx, by)
        sb2 = float(sigma_bias) ** 2
        Q[4, 4] = sb2
        Q[5, 5] = sb2

        return Q

    def predict(self):
        """Time update: x^- = F x, P^- = F P F^T + Q"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        """
        Generic linear KF measurement update.
        z must be shape (m,1) or (m,)
        """
        z = np.asarray(z, dtype=float).reshape(-1, 1)

        # Innovation
        y = z - (H @ self.x)

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain (solve is more stable than explicit inverse)
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))

        # State and covariance update (Joseph form for numerical stability)
        self.x = self.x + K @ y
        I_KH = self.I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def update_velocity(self, vx: float, vy: float):
        """High-rate velocity update."""
        self._update(np.array([vx, vy]), self.H_vel, self.R_vel)

    def update_gps(self, px: float, py: float):
        """Low-rate GPS update (with bias absorption)."""
        self._update(np.array([px, py]), self.H_gps, self.R_gps)

    @property
    def position(self) -> Tuple[float, float]:
        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[2, 0]), float(self.x[3, 0])

    @property
    def gps_bias(self) -> Tuple[float, float]:
        return float(self.x[4, 0]), float(self.x[5, 0])
