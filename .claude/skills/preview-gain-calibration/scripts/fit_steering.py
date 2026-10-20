#!/usr/bin/env python3
"""Fit steering actuator dynamics from step / chirp excitation logs or ordinary by-wire driving.

Model (steering-wheel angle, rad):
    y(t) = lag_tau( ratelimit_R( K * cmd(t - delay) ) ) + offset
cmd  = /mcity/vehicle_control steering_cmd   (what preview_control publishes)
y    = /mcity/vehicle_state  steer_state     (what preview_control reads back)
so the fitted delay is the full loop delay the controller experiences (bridge timer included).

    python fit_steering.py <csv_dir> [<csv_dir> ...] [--params mache.json --update] [--plot out.png]

<csv_dir> = output of bag_to_csv.py. Only samples with by-wire enabled and no override are used.
Each run is split into by-wire segments (>= --min-seg s) whose command moves by >= --min-pp deg, so long
drives with preview_control (or any controller) steering work too; quiet stretches are dropped.
"""
import argparse
import csv
import json
import os

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

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


def load_run(d, min_seg=3.0, min_pp_deg=5.0):
    """-> list of (t, cmd, y) segments of usable, excited by-wire data."""
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
    ok &= np.isfinite(cmd) & np.isfinite(y)
    segs = []
    edges = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a, b in zip(edges[::2], edges[1::2]):
        if (b - a) * GRID_DT >= min_seg and np.degrees(np.ptp(cmd[a:b])) >= min_pp_deg:
            segs.append((t[a:b], cmd[a:b], y[a:b]))
    return segs


def simulate(cmd, delay, tau, K, offset, rate, y0=None):
    """y0: measured angle at the first sample (actuator state); default = settled on cmd[0]."""
    nd = int(round(delay / GRID_DT))
    u = np.concatenate([np.full(nd, cmd[0]), cmd[:len(cmd) - nd]]) * K
    x = u[0] if y0 is None else y0 - offset   # lagged angle
    a = GRID_DT / (tau + GRID_DT)
    dmax = rate * GRID_DT
    if np.all(np.abs(np.diff(u)) <= dmax):    # rate limit never binds -> plain first-order lag
        y, _ = lfilter([a], [1, a - 1], u, zi=[(1 - a) * x])
        return y + offset
    y = np.empty_like(u)
    r = u[0]          # rate-limited command
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
        for t, cmd, y in runs:
            res.append(simulate(cmd, delay, tau, K, off, R, y[0]) - y)
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
    ap.add_argument("--min-seg", type=float, default=3.0, help="min usable segment length [s]")
    ap.add_argument("--min-pp", type=float, default=5.0, help="min command peak-to-peak in a segment [deg]")
    ap.add_argument("--params", help="vehicle params JSON to update")
    ap.add_argument("--update", action="store_true", help="write tau/delay into --params")
    ap.add_argument("--plot", help="save a cmd/measured/model plot (needs matplotlib)")
    a = ap.parse_args()

    runs = [seg for d in a.csv_dirs for seg in load_run(d, a.min_seg, a.min_pp)]
    n_ok = sum(len(r[0]) for r in runs)
    print(f"{len(a.csv_dirs)} run(s), {len(runs)} segment(s), {n_ok * GRID_DT:.1f}s of usable by-wire data")
    if n_ok * GRID_DT < 10:
        raise SystemExit("not enough by-wire steering activity to fit (need >= 10 s); drive more curves under by-wire")
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
        show = sorted(runs, key=lambda r: -np.ptp(r[1]))[:6]   # most excited segments
        fig, axs = plt.subplots(len(show), 1, figsize=(12, 3.5 * len(show)), squeeze=False)
        for ax, (t, cmd, y) in zip(axs[:, 0], show):
            m = simulate(cmd, res["delay"], res["tau"], res["K"], res["offset_rad"], res["rate_rad_s"], y[0])
            ax.plot(t - t[0], np.degrees(cmd), label="cmd")
            ax.plot(t - t[0], np.degrees(y), label="measured")
            ax.plot(t - t[0], np.degrees(m), "--", label="model")
            ax.set_ylabel("steering wheel [deg]"); ax.legend(); ax.grid(True)
        axs[-1, 0].set_xlabel("t [s]")
        fig.tight_layout(); fig.savefig(a.plot, dpi=100)
        print(f"plot: {a.plot}")


if __name__ == "__main__":
    main()
