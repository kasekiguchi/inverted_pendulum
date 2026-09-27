"""Parameter identification from logged experiments."""

from __future__ import annotations

import numpy as np
from scipy.optimize import curve_fit, minimize_scalar


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def fit_swing(t: np.ndarray, th_hang: np.ndarray) -> dict:
    """Fit a damped oscillation to the hanging pendulum angle [rad].

    th(t) = A exp(-s t) cos(wd t + phi) + c,  wn = sqrt(wd^2 + s^2), zeta = s / wn
    """
    t = t - t[0]
    y = th_hang - np.mean(th_hang)
    # initial guess of wd from the FFT peak
    dt = np.median(np.diff(t))
    f = np.fft.rfftfreq(len(y), dt)
    spec = np.abs(np.fft.rfft(y))
    spec[0] = 0
    wd0 = 2 * np.pi * f[np.argmax(spec)]
    A0 = np.max(np.abs(y[: max(1, len(y) // 10)]))

    def model(t, A, s, wd, phi, c):
        return A * np.exp(-s * t) * np.cos(wd * t + phi) + c

    popt, _ = curve_fit(model, t, th_hang, p0=[A0, 0.1, wd0, 0.0, np.mean(th_hang)], maxfev=20000)
    A, s, wd, phi, c = popt
    if A < 0:
        A, phi = -A, phi + np.pi
    wn = np.hypot(wd, s)
    return {
        "wn": wn,
        "zeta": s / wn,
        "offset": c,
        "fit": model(t, A, s, wd, phi, c),
        "rms": float(np.sqrt(np.mean((model(t, *popt) - th_hang) ** 2))),
    }


def simulate_first_order(t, u, tau, delay_steps=0):
    """Position for v' = (u(t - d) - v) / tau, v(0) = p(0) = 0, ZOH input."""
    dt = np.diff(t, prepend=t[0])
    ud = np.concatenate([np.zeros(delay_steps), u[: len(u) - delay_steps]])
    v = p = 0.0
    ps = np.zeros_like(t)
    for k in range(1, len(t)):
        a = np.exp(-dt[k] / tau)
        u_k = ud[k - 1]
        p += u_k * dt[k] + (v - u_k) * tau * (1 - a)
        v = u_k + (v - u_k) * a
        ps[k] = p
    return ps


def fit_step(t: np.ndarray, u: np.ndarray, p: np.ndarray, max_delay: int = 5) -> dict:
    """Fit the speed-loop time constant tau (and an integer sample delay) to the measured angle."""
    t = t - t[0]
    p = p - p[0]
    best = None
    for d in range(max_delay + 1):
        res = minimize_scalar(
            lambda tau: np.sum((simulate_first_order(t, u, tau, d) - p) ** 2),
            bounds=(1e-3, 1.0),
            method="bounded",
        )
        if best is None or res.fun < best[2]:
            best = (res.x, d, res.fun)
    tau, d, _ = best
    fit = simulate_first_order(t, u, tau, d)
    return {"tau": tau, "delay_steps": d, "fit": fit, "rms": float(np.sqrt(np.mean((fit - p) ** 2)))}


def simulate_second_order(t, u, k, wn, zeta, tz, delay_steps=0):
    """Angle response for psi'/u = k wn^2 (tz s + 1) / (s^2 + 2 zeta wn s + wn^2), exact ZOH per sample."""
    from scipy.linalg import expm

    A = np.array([[0, 0, k * wn**2, k * wn**2 * tz], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, -(wn**2), -2 * zeta * wn]])
    # state [psi, (unused), z, z']; input enters z''
    A = np.delete(np.delete(A, 1, 0), 1, 1)
    B = np.array([0.0, 0.0, 1.0])
    ud = np.concatenate([np.zeros(delay_steps), u[: len(u) - delay_steps]])
    cache = {}
    x = np.zeros(3)
    out = np.zeros_like(t)
    for i in range(1, len(t)):
        h = round(float(t[i] - t[i - 1]), 4)
        if h not in cache:
            M = np.zeros((4, 4))
            M[:3, :3], M[:3, 3] = A, B
            E = expm(M * h)
            cache[h] = (E[:3, :3], E[:3, 3])
        Ad, Bd = cache[h]
        x = Ad @ x + Bd * ud[i - 1]
        out[i] = x[0]
    return out


def fit_step2(t: np.ndarray, u: np.ndarray, p: np.ndarray, max_delay: int = 3) -> dict:
    """Fit a second-order speed loop (with a zero) to the measured wheel angle."""
    from scipy.optimize import least_squares

    t = t - t[0]
    p = p - p[0]
    best = None
    for d in range(max_delay + 1):
        res = least_squares(
            lambda x: simulate_second_order(t, u, *x, d) - p,
            x0=[1.0, 60.0, 0.3, 0.01],
            bounds=([0.5, 5.0, 0.02, -0.1], [1.5, 500.0, 3.0, 0.2]),
        )
        if best is None or res.cost < best[2]:
            best = (res.x, d, res.cost)
    (k, wn, zeta, tz), d, _ = best
    fit = simulate_second_order(t, u, k, wn, zeta, tz, d)
    return {"k": k, "wn": wn, "zeta": zeta, "tz": tz, "delay_steps": d, "fit": fit,
            "rms": float(np.sqrt(np.mean((fit - p) ** 2)))}
