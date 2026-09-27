"""Two-wheeled inverted pendulum driven by speed-controlled Roller485 units.

State x = [th, psi, dth, dpsi], input u = wheel speed command relative to the body [rad/s].

    th  : body tilt from upright (+ forward)
    psi : wheel angle relative to the body (+ rolls forward); travel = r (th + psi)

Lagrangian with psi prescribed by the speed loop (psi'' = (u - psi') / tau):

    Jt(th) th'' = m g l sin th + m r l sin th th'^2 - Bt(th) psi''
    Jt(th) = W + m r^2 + 2 m r l cos th + J_axle
    Bt(th) = W + m r^2 + m r l cos th
    W = I_w + M_w r^2,  J_axle = J_body + m l^2

With l -> 0 the translation becomes uncontrollable (th' Jt + psi' Bt is conserved),
so a small l means large lean angles are needed to accelerate.
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.linalg import expm, solve_discrete_are


@dataclass
class Params:
    g: float
    wheel_radius: float
    m_body: float
    l: float
    J_body: float
    swing_wn: float
    m_wheels: float
    motor_tau: float
    dt: float
    tf: float
    u_max: float
    q: list[float]
    r: float
    th0_deg: float
    theta_bias_deg: float
    t_end: float
    enc_res_deg: float

    @classmethod
    def load(cls, path: str | Path = "params.toml") -> Params:
        with open(path, "rb") as f:
            d = tomllib.load(f)
        return cls(**d["plant"], **d["control"], **d["lqr"], **d["sim"])

    # derived quantities -------------------------------------------------
    @property
    def J_axle(self) -> float:
        """Body inertia about the axle; from the upside-down swing test if available."""
        if self.swing_wn > 0:
            return self.m_body * self.g * self.l / self.swing_wn**2
        return self.J_body + self.m_body * self.l**2

    @property
    def W(self) -> float:
        rw = self.wheel_radius
        return 1.5 * self.m_wheels * rw**2  # solid discs: I_w + M_w r^2

    def Jt(self, th=0.0):
        m, r, l = self.m_body, self.wheel_radius, self.l
        return self.W + m * r**2 + 2 * m * r * l * np.cos(th) + self.J_axle

    def Bt(self, th=0.0):
        m, r, l = self.m_body, self.wheel_radius, self.l
        return self.W + m * r**2 + m * r * l * np.cos(th)

    def lean_per_accel(self) -> float:
        """Steady lean [rad] needed per 1 m/s^2 of forward acceleration."""
        return self.Bt() / (self.m_body * self.g * self.l * self.wheel_radius)


def linear_model(p: Params) -> tuple[np.ndarray, np.ndarray]:
    a = p.m_body * p.g * p.l / p.Jt()
    b = p.Bt() / p.Jt()
    tau = p.motor_tau
    A = np.array(
        [
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [a, 0, 0, b / tau],
            [0, 0, 0, -1 / tau],
        ]
    )
    B = np.array([[0], [0], [-b / tau], [1 / tau]])
    return A, B


def discretize(A: np.ndarray, B: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Zero-order-hold discretization."""
    n, m = B.shape
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n:]


def dlqr(Ad, Bd, Q, R) -> np.ndarray:
    P = solve_discrete_are(Ad, Bd, Q, R)
    return np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)


def design(p: Params) -> tuple[np.ndarray, np.ndarray]:
    """Return (K, closed-loop discrete poles) for u = -K x."""
    Ad, Bd = discretize(*linear_model(p), p.dt)
    K = dlqr(Ad, Bd, np.diag(p.q), np.atleast_2d(p.r))
    return K.ravel(), np.linalg.eigvals(Ad - Bd @ K)


def dynamics(p: Params, x: np.ndarray, u: float) -> np.ndarray:
    th, _, dth, dpsi = x
    m, r, l = p.m_body, p.wheel_radius, p.l
    ddpsi = (u - dpsi) / p.motor_tau
    ddth = (m * p.g * l * np.sin(th) + m * r * l * np.sin(th) * dth**2 - p.Bt(th) * ddpsi) / p.Jt(th)
    return np.array([dth, dpsi, ddth, ddpsi])


def simulate(p: Params, K: np.ndarray, x0: np.ndarray, t_end: float, substeps: int = 10) -> dict:
    """Nonlinear simulation reproducing the firmware loop: tilt measured with a
    constant bias (trim error), quantised wheel encoders, filtered-derivative
    wheel speed, saturation and ZOH."""
    h = p.dt
    n = int(round(t_end / h))
    q_psi = np.deg2rad(p.enc_res_deg)
    bias = np.deg2rad(p.theta_bias_deg)

    x = np.array(x0, dtype=float)
    psim_old = np.round(x[1] / q_psi) * q_psi
    dpsih = 0.0
    names = ("t", "th", "psi", "dth", "dpsi", "u", "travel")
    log = {k: np.zeros(n) for k in names}
    for k in range(n):
        psim = np.round(x[1] / q_psi) * q_psi
        dpsih = (2 * (psim - psim_old) + (2 * p.tf - h) * dpsih) / (2 * p.tf + h)
        psim_old = psim
        xh = np.array([x[0] + bias, psim, x[2], dpsih])
        u = float(np.clip(-K @ xh, -p.u_max, p.u_max))
        for name, val in zip(names, (k * h, *x, u, p.wheel_radius * (x[0] + x[1]))):
            log[name][k] = val
        dt = h / substeps
        for _ in range(substeps):  # RK4
            k1 = dynamics(p, x, u)
            k2 = dynamics(p, x + dt / 2 * k1, u)
            k3 = dynamics(p, x + dt / 2 * k2, u)
            k4 = dynamics(p, x + dt * k3, u)
            x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if abs(x[0]) > np.pi / 2:
            for name in names:
                log[name] = log[name][: k + 1]
            log["fell"] = True
            return log
    log["fell"] = False
    return log
