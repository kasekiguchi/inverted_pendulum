"""Cart inverted pendulum driven by a speed-controlled Roller485.

State x = [p, th, v, dth], input u = cart velocity command [m/s].

    p'   = v
    v'   = (u - v) / tau
    th'' = wn^2 sin(th) - 2 zeta wn th' - (wn^2 / g) cos(th) v'

wn and zeta are the hanging-pendulum natural frequency and damping ratio, so
all pendulum parameters (mass, length, inertia) fold into wn^2 = m l g / J.
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
    pend_wn: float
    pend_zeta: float
    cart_tau: float
    dt: float
    tf: float
    u_max: float
    q: list[float]
    r: float
    th0_deg: float
    t_end: float
    enc_res_deg: float

    @classmethod
    def load(cls, path: str | Path = "params.toml") -> Params:
        with open(path, "rb") as f:
            d = tomllib.load(f)
        return cls(**d["plant"], **d["control"], **d["lqr"], **d["sim"])


def linear_model(p: Params) -> tuple[np.ndarray, np.ndarray]:
    a = p.pend_wn**2
    c = 2 * p.pend_zeta * p.pend_wn
    tau = p.cart_tau
    A = np.array(
        [
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, -1 / tau, 0],
            [0, a, a / p.g / tau, -c],
        ]
    )
    B = np.array([[0], [0], [1 / tau], [-a / p.g / tau]])
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
    _, th, v, dth = x
    a = p.pend_wn**2
    acc = (u - v) / p.cart_tau
    ddth = a * np.sin(th) - 2 * p.pend_zeta * p.pend_wn * dth - a / p.g * np.cos(th) * acc
    return np.array([v, dth, acc, ddth])


def simulate(p: Params, K: np.ndarray, x0: np.ndarray, t_end: float, substeps: int = 10) -> dict:
    """Nonlinear simulation reproducing the firmware loop: quantised encoders,
    filtered-derivative velocity estimates, saturation and ZOH."""
    h = p.dt
    n = int(round(t_end / h))
    q_th = np.deg2rad(p.enc_res_deg)
    q_p = q_th * p.wheel_radius

    def quant(v, q):
        return np.round(v / q) * q

    def deriv(y, x_new, x_old):
        return (2 * (x_new - x_old) + (2 * p.tf - h) * y) / (2 * p.tf + h)

    x = np.array(x0, dtype=float)
    pm_old, thm_old = quant(x[0], q_p), quant(x[1], q_th)
    vh = dthh = 0.0
    log = {k: np.zeros(n) for k in ("t", "p", "th", "v", "dth", "u", "vh", "dthh")}
    for k in range(n):
        pm, thm = quant(x[0], q_p), quant(x[1], q_th)
        vh, dthh = deriv(vh, pm, pm_old), deriv(dthh, thm, thm_old)
        pm_old, thm_old = pm, thm
        xh = np.array([pm, thm, vh, dthh])
        u = float(np.clip(-K @ xh, -p.u_max, p.u_max))
        for name, val in zip(log, (k * h, *x, u, vh, dthh)):
            log[name][k] = val
        dt = h / substeps
        for _ in range(substeps):  # RK4
            k1 = dynamics(p, x, u)
            k2 = dynamics(p, x + dt / 2 * k1, u)
            k3 = dynamics(p, x + dt / 2 * k2, u)
            k4 = dynamics(p, x + dt * k3, u)
            x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if abs(x[1]) > np.pi / 2:
            for name in log:
                log[name] = log[name][: k + 1]
            log["fell"] = True
            return log
    log["fell"] = False
    return log
