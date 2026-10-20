#!/usr/bin/env python3
"""Fit steering actuator dynamics from step / chirp excitation logs.

Model (steering-wheel angle, rad):
    y(t) = lag_tau( ratelimit_R( K * cmd(t - delay) ) ) + offset
cmd  = /mcity/vehicle_control steering_cmd   (what preview_control publishes)
y    = /mcity/vehicle_state  steer_state     (what preview_control reads back)
so the fitted delay is the full loop delay the controller experiences (bridge timer included).

    python fit_steering.py <csv_dir> [<csv_dir> ...] [--params mache.json --update] [--plot out.png]

<csv_dir> = output of bag_to_csv.py. Only samples with by-wire enabled and no override are used.
"""
import argparse
import csv
import json
import os

import numpy as np
from scipy.optimize import least_squares

GRID_DT = 0.01


def load_csv(path):
    """CSV (from bag_to_csv.py) -> {column: float array}; non-numeric cells become NaN."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    out = {}
    for j, h in enumerate(rows[0]):
        col = []
        for r in rows[1:]:
            try:
                col.append(float(r[j]))
            except (ValueError, IndexError):
                col.append(np.nan)
        out[h] = np.array(col)
    return out


def load_run(d):
    ctrl = load_csv(os.path.join(d, "__mcity__vehicle_control.csv"))
    vs = load_csv(os.path.join(d, "__mcity__vehicle_state.csv"))
    t0, t1 = max(ctrl["t"][0], vs["t"][0]), min(ctrl["t"][-1], vs["t"][-1])
    t = np.arange(t0, t1, GRID_DT)
    # zero-order hold for the command, linear interpolation for the measurement
    idx = np.clip(np.searchsorted(ctrl["t"], t, side="right") - 1, 0, len(ctrl["t"]) - 1)
    cmd = ctrl["steering_cmd"][idx]
    y = np.interp(t, vs["t"], vs["steer_state"])
    ok = np.ones_like(t, dtype=bool)
    for f in ("steer_override", "brake_override", "throttle_override"):
        if f in vs:
            ok &= np.interp(t, vs["t"], vs[f]) < 0.5
    if "by_wire_enabled" in vs:
        ok &= np.interp(t, vs["t"], vs["by_wire_enabled"]) > 0.5
    # ignore samples older than 0.5 s since the last command (controller not running)
    age = t - ctrl["t"][idx]
    ok &= age < 0.5
    return t, cmd, y, ok


def simulate(cmd, delay, tau, K, offset, rate):
    nd = int(round(delay / GRID_DT))
    u = np.concatenate([np.full(nd, cmd[0]), cmd[:len(cmd) - nd]]) * K
    y = np.empty_like(u)
    r = u[0]          # rate-limited command
    x = u[0]          # lagged angle
    a = GRID_DT / (tau + GRID_DT)
    dmax = rate * GRID_DT
    for i in range(len(u)):
        r += max(-dmax, min(dmax, u[i] - r))
        x += a * (r - x)
        y[i] = x
    return y + offset


def fit(runs, rate, fit_rate):
    def resid(theta, delay):
        tau, K, off = np.exp(theta[0]), theta[1], theta[2]
        R = np.exp(theta[3]) if fit_rate else rate
        res = []
        for t, cmd, y, ok in runs:
            res.append((simulate(cmd, delay, tau, K, off, R) - y)[ok])
        return np.concatenate(res)

    x0 = [np.log(0.1), 1.0, 0.0] + ([np.log(rate)] if fit_rate else [])
    best = None
    for delay in np.arange(0.0, 0.50 + 1e-9, GRID_DT):
        sol = least_squares(resid, x0, args=(delay,), max_nfev=60)
        cost = np.mean(sol.fun ** 2)
        if best is None or cost < best[0]:
            best = (cost, delay, sol.x)
    cost, delay, th = best
    out = {"delay": float(delay), "tau": float(np.exp(th[0])), "K": float(th[1]),
           "offset_rad": float(th[2]), "rate_rad_s": float(np.exp(th[3])) if fit_rate else rate,
           "rms_deg": float(np.degrees(np.sqrt(cost)))}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_dirs", nargs="+")
    ap.add_argument("--rate-deg", type=float, default=300.0,
                    help="steering-wheel rate limit [deg/s] (mcity_bywire_node cmd_rate)")
    ap.add_argument("--fit-rate", action="store_true", help="also fit the rate limit")
    ap.add_argument("--params", help="vehicle params JSON to update")
    ap.add_argument("--update", action="store_true", help="write tau/delay into --params")
    ap.add_argument("--plot", help="save a cmd/measured/model plot (needs matplotlib)")
    a = ap.parse_args()

    runs = [load_run(d) for d in a.csv_dirs]
    n_ok = sum(int(r[3].sum()) for r in runs)
    print(f"{len(runs)} run(s), {n_ok * GRID_DT:.1f}s of usable by-wire data")
    res = fit(runs, np.radians(a.rate_deg), a.fit_rate)

    print(f"delay        : {res['delay']:.3f} s  (= {res['delay'] / 0.04:.1f} controller steps of 0.04 s; buffer holds 5)")
    print(f"lag tau      : {res['tau']:.3f} s")
    print(f"tracking K   : {res['K']:.3f}  (steady steer_state / steering_cmd; 1.0 = perfect)")
    print(f"offset       : {np.degrees(res['offset_rad']):.2f} deg steering wheel")
    print(f"rate limit   : {np.degrees(res['rate_rad_s']):.0f} deg/s {'(fitted)' if a.fit_rate else '(fixed)'}")
    print(f"fit rms      : {res['rms_deg']:.2f} deg steering wheel")
    if abs(res["K"] - 1) > 0.1:
        print("NOTE: K far from 1 -> the by-wire angle loop does not track; this is what the "
              "'steering * 1.4' in preview_control.cpp publishCmd() was compensating. Fix at the source "
              "(DBW tuning) or keep a documented gain = 1/K.")
    if res["delay"] > 0.2 + 1e-6:
        print("WARNING: delay > 0.2 s exceeds the 5-slot delay buffer in path_follow.cpp; gains will be clipped.")

    if a.params and a.update:
        p = json.load(open(a.params))
        p["tau"] = round(res["tau"], 4)
        p["delay"] = round(res["delay"], 3)
        p.setdefault("_measured", {})["steering"] = {k: round(v, 5) for k, v in res.items()}
        json.dump(p, open(a.params, "w", newline="\n"), indent=2)
        print(f"updated {a.params}")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(runs), 1, figsize=(12, 3.5 * len(runs)), squeeze=False)
        for ax, (t, cmd, y, ok) in zip(axs[:, 0], runs):
            m = simulate(cmd, res["delay"], res["tau"], res["K"], res["offset_rad"], res["rate_rad_s"])
            ax.plot(t - t[0], np.degrees(cmd), label="cmd")
            ax.plot(t - t[0], np.degrees(y), label="measured")
            ax.plot(t - t[0], np.degrees(m), "--", label="model")
            ax.set_ylabel("steering wheel [deg]"); ax.legend(); ax.grid(True)
        axs[-1, 0].set_xlabel("t [s]")
        fig.tight_layout(); fig.savefig(a.plot, dpi=100)
        print(f"plot: {a.plot}")


if __name__ == "__main__":
    main()
