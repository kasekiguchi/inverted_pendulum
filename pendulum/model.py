"""Two-wheeled inverted pendulum driven by speed-controlled Roller485 units.

State x = [th, psi, dth, <motor states>], input u = wheel speed command relative to the body [rad/s].

    th  : body tilt from upright (+ forward)
    psi : wheel angle relative to the body (+ rolls forward); travel = r (th + psi)

The Roller485 speed loop (u -> psi') is a linear system (Am, Bm, Cm):
    first order : psi' = w,  w' = (u - w) / motor_tau
    second order: psi'/u = k wn^2 (tz s + 1) / (s^2 + 2 zeta wn s + wn^2)
The controller measures [th, psi, dth, dpsi] only.

Lagrangian with psi prescribed by the speed loop:

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
from scipy.linalg import expm, solve_discrete_are, solve_discrete_lyapunov
from scipy.optimize import minimize


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
    # identified second-order speed loop (motor_wn = 0: use motor_tau only)
    motor_wn: float = 0.0
    motor_zeta: float = 1.0
    motor_tz: float = 0.0
    motor_k: float = 1.0
    motor_delay: int = 0  # input dead time [samples]

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

    @property
    def has_motor2(self) -> bool:
        return self.motor_wn > 0


def motor_ss(p: Params, second: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Speed loop u -> psi' as (Am, Bm, Cm); psi' = Cm xm, no feedthrough."""
    if second:
        wn, z, tz, k = p.motor_wn, p.motor_zeta, p.motor_tz, p.motor_k
        # controllable canonical form scaled by diag(wn^2, wn) for conditioning
        Am = np.array([[0.0, wn], [-wn, -2 * z * wn]])
        Bm = np.array([[0.0], [wn]])
        Cm = np.array([[k, k * tz * wn]])
    else:
        Am = np.array([[-1 / p.motor_tau]])
        Bm = np.array([[1 / p.motor_tau]])
        Cm = np.array([[1.0]])
    return Am, Bm, Cm


def linear_model(p: Params, second: bool = False):
    """Return (A, B, C): x = [th, psi, dth, xm], measured y = C x = [th, psi, dth, dpsi]."""
    a = p.m_body * p.g * p.l / p.Jt()
    b = p.Bt() / p.Jt()
    Am, Bm, Cm = motor_ss(p, second)
    nm = Am.shape[0]
    n = 3 + nm
    A = np.zeros((n, n))
    B = np.zeros((n, 1))
    A[0, 2] = 1
    A[1, 3:] = Cm
    A[2, 0] = a
    A[2, 3:] = -b * (Cm @ Am)  # th'' = a th - b psi'',  psi'' = Cm (Am xm + Bm u)
    B[2, 0] = -b * (Cm @ Bm).item()
    A[3:, 3:] = Am
    B[3:, :] = Bm
    C = np.zeros((4, n))
    C[0, 0] = C[1, 1] = C[2, 2] = 1
    C[3, 3:] = Cm
    return A, B, C


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


def discrete_model(p: Params, second: bool, delay: int = 0):
    """ZOH model of what the firmware sees.

    Extra states: `delay` samples of input dead time, and the firmware's
    filtered derivative of psi (previous psi and previous estimate), so the
    measured dpsi is the estimate y_k = c1 (psi_k - psi_{k-1}) + c2 y_{k-1}."""
    A, B, C = linear_model(p, second)
    Ad, Bd = discretize(A, B, p.dt)
    n = Ad.shape[0]
    N = n + delay + 2
    Ada = np.zeros((N, N))
    Bda = np.zeros((N, 1))
    Ada[:n, :n] = Ad
    if delay == 0:
        Bda[:n] = Bd
    else:
        Ada[:n, n + delay - 1] = Bd[:, 0]  # plant sees the oldest stored input
        Bda[n, 0] = 1
        for i in range(delay - 1):
            Ada[n + i + 1, n + i] = 1
    h, tf = p.dt, p.tf
    c1, c2 = 2 / (2 * tf + h), (2 * tf - h) / (2 * tf + h)
    ip, iy = n + delay, n + delay + 1  # psi_{k-1}, y_{k-1}
    est = np.zeros(N)  # y_k as a function of the state
    est[1], est[ip], est[iy] = c1, -c1, c2
    Ada[ip, 1] = 1
    Ada[iy] = est
    Ca = np.zeros((4, N))
    Ca[:3, :n] = C[:3]
    Ca[3] = est
    return Ada, Bda, Ca


def closed_loop_poles(p: Params, K: np.ndarray, second: bool, delay: int = 0) -> np.ndarray:
    """Discrete closed-loop poles with u = -K y (y = measured [th, psi, dth, dpsi])."""
    Ad, Bd, C = discrete_model(p, second, delay)
    return np.linalg.eigvals(Ad - Bd @ K[None, :] @ C)


def design(p: Params) -> tuple[np.ndarray, np.ndarray]:
    """LQR on the first-order motor model. Returns (K, closed-loop discrete poles), u = -K x."""
    A, B, _ = linear_model(p, second=False)
    Ad, Bd = discretize(A, B, p.dt)
    K = dlqr(Ad, Bd, np.diag(p.q), np.atleast_2d(p.r)).ravel()
    return K, np.linalg.eigvals(Ad - Bd @ K[None, :])


def design_output_feedback(p: Params, K0: np.ndarray) -> np.ndarray:
    """Optimise the 4 measured-state gains against the identified motor model
    (second order + dead time).

    Minimises the LQ cost sum(y'Qy + u'Ru) averaged over unit initial states,
    starting from K0 (usually the first-order LQR gain)."""
    Ad, Bd, C = discrete_model(p, second=True, delay=p.motor_delay)
    Q, R = np.diag(p.q), np.atleast_2d(p.r)

    def cost(k):
        K = k[None, :]
        Acl = Ad - Bd @ K @ C
        if np.max(np.abs(np.linalg.eigvals(Acl))) >= 0.999:
            return 1e12
        Qcl = C.T @ (Q + K.T @ R @ K) @ C
        return np.trace(solve_discrete_lyapunov(Acl.T, Qcl))

    def rho(k):
        return np.max(np.abs(np.linalg.eigvals(Ad - Bd @ k[None, :] @ C)))

    if cost(K0) >= 1e12:
        # The LQR gain is unstable on this model: first minimise the spectral radius.
        starts = [minimize(rho, K0 * s, method="Nelder-Mead", options={"maxiter": 4000}) for s in (1.0, 0.5, 0.25)]
        best = min(starts, key=lambda r: r.fun)
        if best.fun >= 0.999:
            raise RuntimeError(f"no stabilising static gain found (best max|z| = {best.fun:.4f})")
        K0 = best.x
    res = minimize(cost, K0, method="Nelder-Mead", options={"xatol": 1e-4, "fatol": 1e-6, "maxiter": 20000})
    return res.x


def dynamics(p: Params, x: np.ndarray, u: float, motor) -> np.ndarray:
    Am, Bm, Cm = motor
    th, _, dth = x[:3]
    xm = x[3:]
    dxm = Am @ xm + Bm[:, 0] * u
    dpsi = (Cm @ xm).item()
    ddpsi = (Cm @ dxm).item()
    m, r, l = p.m_body, p.wheel_radius, p.l
    ddth = (m * p.g * l * np.sin(th) + m * r * l * np.sin(th) * dth**2 - p.Bt(th) * ddpsi) / p.Jt(th)
    return np.concatenate([[dth, dpsi, ddth], dxm])


def simulate(p: Params, K: np.ndarray, th0: float, t_end: float, second: bool | None = None,
             substeps: int = 10) -> dict:
    """Nonlinear simulation reproducing the firmware loop: tilt measured with a
    constant bias (trim error), quantised wheel encoders, filtered-derivative
    wheel speed, saturation and ZOH. Uses the second-order motor if identified."""
    if second is None:
        second = p.has_motor2
    motor = motor_ss(p, second)
    h = p.dt
    n = int(round(t_end / h))
    q_psi = np.deg2rad(p.enc_res_deg)
    bias = np.deg2rad(p.theta_bias_deg)

    x = np.zeros(3 + motor[0].shape[0])
    x[0] = th0
    delay = p.motor_delay if second else 0
    u_queue = [0.0] * delay
    psim_old = 0.0
    dpsih = 0.0
    names = ("t", "th", "psi", "dth", "dpsi", "u", "travel")
    log = {k: np.zeros(n) for k in names}
    for k in range(n):
        psim = np.round(x[1] / q_psi) * q_psi
        dpsih = (2 * (psim - psim_old) + (2 * p.tf - h) * dpsih) / (2 * p.tf + h)
        psim_old = psim
        xh = np.array([x[0] + bias, psim, x[2], dpsih])
        u_cmd = float(np.clip(-K @ xh, -p.u_max, p.u_max))
        u_queue.append(u_cmd)
        u = u_queue.pop(0)
        dpsi = (motor[2] @ x[3:]).item()
        for name, val in zip(names, (k * h, x[0], x[1], x[2], dpsi, u_cmd, p.wheel_radius * (x[0] + x[1]))):
            log[name][k] = val
        dt = h / substeps
        for _ in range(substeps):  # RK4
            k1 = dynamics(p, x, u, motor)
            k2 = dynamics(p, x + dt / 2 * k1, u, motor)
            k3 = dynamics(p, x + dt / 2 * k2, u, motor)
            k4 = dynamics(p, x + dt * k3, u, motor)
            x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if abs(x[0]) > np.pi / 2:
            for name in names:
                log[name] = log[name][: k + 1]
            log["fell"] = True
            return log
    log["fell"] = False
    return log
