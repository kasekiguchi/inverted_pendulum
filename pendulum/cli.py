"""Command line entry point: `pend <command>` (run `pend -h`)."""

from __future__ import annotations

import argparse
import re
from datetime import datetime
from pathlib import Path

import numpy as np

from . import ident
from .device import Device, load_log
from .model import Params, closed_loop_poles, design, design_output_feedback, simulate


def update_toml(path: str, updates: dict[str, float]):
    """Rewrite `key = value` lines in place, keeping comments."""
    text = Path(path).read_text()
    for key, val in updates.items():
        text, n = re.subn(
            rf"^({re.escape(key)}\s*=\s*)[^#\n]*?(\s*(#.*)?)$",
            lambda m: f"{m.group(1)}{val if isinstance(val, int) else f'{val:.4g}'}{m.group(2)}",
            text,
            count=1,
            flags=re.M,
        )
        if n == 0:
            raise KeyError(f"{key} not found in {path}")
    Path(path).write_text(text)
    print(f"updated {path}: " + ", ".join(f"{k}={v:.4g}" for k, v in updates.items()))


def print_poles(z, dt, label):
    s = np.log(z.astype(complex)) / dt
    worst = np.max(np.abs(z))
    print(f"{label}: max|z| = {worst:.4f} {'(UNSTABLE)' if worst >= 1 else ''}")
    for zi, si in sorted(zip(z, s), key=lambda t: -abs(t[0]))[:4]:
        zeta = -si.real / abs(si) if abs(si) > 0 else 1.0
        print(f"  |z|={abs(zi):.4f}  s={si.real:+8.2f}{si.imag:+8.2f}j  zeta={zeta:.2f}")


def gain_command(K) -> str:
    return "K " + " ".join(f"{k:.6g}" for k in K)


def default_log_name(kind: str) -> str:
    Path("logs").mkdir(exist_ok=True)
    return f"logs/{datetime.now():%Y%m%d_%H%M%S}_{kind}.csv"


def cmd_design(a):
    import matplotlib.pyplot as plt

    p = Params.load(a.params)
    K, poles = design(p)
    print("LQR on the first-order motor model, K (u = -K x, x = [th, psi, dth, dpsi]):")
    print("  " + "  ".join(f"{k:+.4f}" for k in K))
    print_poles(poles, p.dt, "  poles (design model)")
    if p.has_motor2:
        print(f"measured motor model: wn={p.motor_wn:.1f} rad/s zeta={p.motor_zeta:.2f} "
              f"tz={p.motor_tz * 1e3:.0f} ms delay={p.motor_delay} samples")
        print_poles(closed_loop_poles(p, K, True, p.motor_delay), p.dt, "  LQR gain on the measured model")
        if not a.lqr_only:
            try:
                K = design_output_feedback(p, K)
            except RuntimeError as e:
                raise SystemExit(str(e))
            print("gains optimised on the measured motor model:")
            print("  " + "  ".join(f"{k:+.4f}" for k in K))
            print_poles(closed_loop_poles(p, K, True, p.motor_delay), p.dt, "  optimised gain on the measured model")
    print("firmware command:\n  " + gain_command(K))

    print(f"lean needed per 1 m/s^2 of acceleration: {np.rad2deg(p.lean_per_accel()):.1f} deg "
          f"(l = {p.l * 1e3:.1f} mm)")

    log = simulate(p, K, np.deg2rad(p.th0_deg), p.t_end)
    sat = np.mean(np.abs(log["u"]) >= p.u_max - 1e-9)
    print(f"simulation ({'measured' if p.has_motor2 else 'first-order'} motor, th0={p.th0_deg} deg, "
          f"tilt bias={p.theta_bias_deg} deg): "
          f"{'FELL' if log['fell'] else 'ok'}, max|travel|={np.max(np.abs(log['travel'])):.3f} m, "
          f"final travel={log['travel'][-1]:+.3f} m, max|u|={np.max(np.abs(log['u'])):.1f} rad/s, "
          f"saturated {sat:.0%} of samples")

    if a.send:
        with Device(a.send) as dev:
            dev.cmd(gain_command(K))
            dev.cmd(f"TF {p.tf}")
            dev.cmd(f"UMAX {p.u_max}")
            dev.cmd(f"R {p.wheel_radius}")
            if a.save:
                dev.cmd("SAVE")
    if not a.no_plot:
        plot_states(log, f"simulation (th0={p.th0_deg} deg)")
        plt.show()


def plot_states(log, title):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
    t = log["t"]
    ax[0].plot(t, np.rad2deg(log["th"]), label="th")
    if "th_acc" in log:
        ax[0].plot(t, np.rad2deg(log["th_acc"]), alpha=0.5, label="th_acc")
        ax[0].legend()
    ax[0].set_ylabel("th [deg]")
    ax[1].plot(t, np.rad2deg(log["psi"]))
    ax[1].set_ylabel("psi [deg]")
    ax[2].plot(t, np.rad2deg(log["dth"]))
    ax[2].set_ylabel("dth [deg/s]")
    ax[3].plot(t, log["u"])
    ax[3].set_ylabel("u [rad/s]")
    ax[3].set_xlabel("t [s]")
    for x in ax:
        x.grid(True)
    fig.suptitle(title)
    fig.tight_layout()
    return fig


def cmd_term(a):
    with Device(a.port) as dev:
        dev.terminal()


def cmd_send(a):
    with Device(a.port) as dev:
        for c in a.commands:
            dev.cmd(c, timeout=5.0)


def cmd_log(a):
    out = a.out or default_log_name(a.kind)
    with Device(a.port) as dev:
        n = dev.record(out, a.duration, a.cmd)
        dev.cmd("STOP", echo=False)
    print(f"wrote {n} rows to {out}")


def cmd_run(a):
    """Arm the controller and record until the duration ends."""
    out = a.out or default_log_name("run")
    with Device(a.port) as dev:
        print("ARMED: bring the robot upright; balancing starts automatically.")
        n = dev.record(out, a.duration, ["ARM"])
        dev.cmd("STOP", echo=False)
    print(f"wrote {n} rows to {out}")


def cmd_plot(a):
    import matplotlib.pyplot as plt

    log = load_log(a.csv)
    fig = plot_states(log, a.csv)
    ex = log["exec_us"]
    print(f"exec time: mean {ex.mean():.0f} us, max {ex.max():.0f} us")
    dt = np.diff(log["t_ms"])
    print(f"sample period: mean {dt.mean():.2f} ms, max {dt.max():.0f} ms (gaps>1.5dt: {(dt > 1.5 * np.median(dt)).sum()})")
    plt.show()


def cmd_fit_swing(a):
    import matplotlib.pyplot as plt

    log = load_log(a.csv)
    t, th = log["t"], ident.wrap(log["th"] + np.pi)  # upside down: angle from hanging
    m = (t >= a.t0) & (t <= (a.t1 if a.t1 else np.inf))
    r = ident.fit_swing(t[m], th[m])
    print(f"swing_wn = {r['wn']:.4f} rad/s ({r['wn'] / 2 / np.pi:.3f} Hz), zeta = {r['zeta']:.4f}")
    print(f"hanging offset = {np.rad2deg(r['offset']):+.3f} deg (the CoG is off the body axis by this angle)")
    print(f"fit rms = {np.rad2deg(r['rms']):.3f} deg")
    if a.update:
        update_toml(a.params, {"swing_wn": r["wn"]})
    plt.plot(t[m], np.rad2deg(th[m]), ".", ms=2, label="measured")
    plt.plot(t[m], np.rad2deg(r["fit"]), label="fit")
    plt.xlabel("t [s]")
    plt.ylabel("angle from hanging [deg]")
    plt.legend()
    plt.grid(True)
    plt.show()


def cmd_fit_step(a):
    import matplotlib.pyplot as plt

    log = load_log(a.csv)
    m = log["state"] == 3  # STEP
    if not m.any():
        raise SystemExit("no STEP samples in log")
    t, u, p = log["t"][m], log["u"][m], log["psi"][m]
    r1 = ident.fit_step(t, u, p)
    print(f"first order : tau = {r1['tau'] * 1e3:.1f} ms, delay = {r1['delay_steps']} samples, "
          f"fit rms = {np.rad2deg(r1['rms']):.2f} deg")
    r2 = ident.fit_step2(t, u, p)
    print(f"second order: k = {r2['k']:.3f}, wn = {r2['wn']:.1f} rad/s, zeta = {r2['zeta']:.3f}, "
          f"tz = {r2['tz'] * 1e3:.1f} ms, delay = {r2['delay_steps']} samples, fit rms = {np.rad2deg(r2['rms']):.2f} deg")
    if a.update:
        # the design model is first order; take its time constant from the second-order fit's
        # 63 % rise so a wrong first-order fit (e.g. tau at the bound) does not leak in.
        tau = max(r1["tau"], 1.0 / r2["wn"])
        update_toml(a.params, {"motor_tau": tau, "motor_k": r2["k"], "motor_wn": r2["wn"],
                               "motor_zeta": r2["zeta"], "motor_tz": r2["tz"],
                               "motor_delay": int(r2["delay_steps"])})
    tt = t - t[0]
    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(tt, p - p[0], ".", ms=2, label="measured")
    ax[0].plot(tt, r1["fit"], label="first order")
    ax[0].plot(tt, r2["fit"], label="second order")
    ax[0].set_ylabel("psi [rad]")
    ax[0].legend()
    ax[1].plot(tt, u)
    ax[1].set_ylabel("u [rad/s]")
    ax[1].set_xlabel("t [s]")
    for x in ax:
        x.grid(True)
    plt.show()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="pend", description=__doc__)
    ap.add_argument("--params", default="params.toml")
    sub = ap.add_subparsers(required=True)

    s = sub.add_parser("design", help="LQR design + nonlinear simulation")
    s.add_argument("--send", metavar="PORT", help="send K/TF/UMAX/R to the device")
    s.add_argument("--save", action="store_true", help="also SAVE to device flash")
    s.add_argument("--no-plot", action="store_true")
    s.add_argument("--lqr-only", action="store_true", help="skip the optimisation on the measured motor model")
    s.set_defaults(func=cmd_design)

    s = sub.add_parser("term", help="interactive serial console")
    s.add_argument("port")
    s.set_defaults(func=cmd_term)

    s = sub.add_parser("send", help="send commands, e.g. pend send /dev/ttyUSB0 INFO GET")
    s.add_argument("port")
    s.add_argument("commands", nargs="+")
    s.set_defaults(func=cmd_send)

    s = sub.add_parser("log", help="record a log (optionally after sending commands)")
    s.add_argument("port")
    s.add_argument("-d", "--duration", type=float, default=10.0)
    s.add_argument("-c", "--cmd", action="append", default=[], help='e.g. -c "STEP 10 1.0"')
    s.add_argument("-k", "--kind", default="log", help="tag used in the default file name")
    s.add_argument("-o", "--out")
    s.set_defaults(func=cmd_log)

    s = sub.add_parser("run", help="ARM and record a balancing run")
    s.add_argument("port")
    s.add_argument("-d", "--duration", type=float, default=20.0)
    s.add_argument("-o", "--out")
    s.set_defaults(func=cmd_run)

    s = sub.add_parser("plot", help="plot a recorded log")
    s.add_argument("csv")
    s.set_defaults(func=cmd_plot)

    s = sub.add_parser("fit-swing", help="identify swing_wn from an upside-down swing log")
    s.add_argument("csv")
    s.add_argument("--t0", type=float, default=0.0)
    s.add_argument("--t1", type=float)
    s.add_argument("--update", action="store_true", help="write results to params.toml")
    s.set_defaults(func=cmd_fit_swing)

    s = sub.add_parser("fit-step", help="identify motor_tau from a STEP log (wheels in the air)")
    s.add_argument("csv")
    s.add_argument("--update", action="store_true", help="write results to params.toml")
    s.set_defaults(func=cmd_fit_step)

    a = ap.parse_args(argv)
    a.func(a)


if __name__ == "__main__":
    main()
