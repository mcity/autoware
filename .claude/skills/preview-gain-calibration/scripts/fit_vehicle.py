#!/usr/bin/env python3
"""Fit steering ratio, understeer gradient and axle cornering stiffness from driving logs.

    python fit_vehicle.py --params mache.json --steady <csv_dir> ... [--transient <csv_dir> ...] [--update]

--steady     constant-steer circles (Test 2). Gives steering ratio and understeer gradient Kus:
                 steer_sw = ratio * (L * kappa + Kus * a_y) + offset,  kappa = r / v,  a_y = v * r
--transient  sine-steer runs at speed (Test 3). Splits Kus into front/rear cornering stiffness
             Cf, Cr by matching the measured yaw rate with a linear bicycle model.
             Without it, Cr is assumed from --cr-norm and Cf solved from Kus (less accurate).

Signals (from bag_to_csv.py):
  steering wheel : /mcity/vehicle_state steer_state [rad]
  yaw rate       : /ins/imu angular_velocity.z   (fallback: d/dt of /mcity/cav_pose yaw)
  speed          : |/ins/odometry twist.linear|  (fallback: vehicle_state speed_x * --speed-scale)
Also reports the wheel-speed scale error of speed_x (the by-wire node uses the MKZ wheel diameter).
"""
import argparse
import csv
import json
import os

import numpy as np
from scipy.optimize import least_squares

G = 9.81
DT = 0.02


def load(path):
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


def yaw_from_quat(qx, qy, qz, qw):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def smooth(x, n):
    if n <= 1:
        return x
    k = np.ones(n) / n
    return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode="edge"), k, mode="valid")


def load_run(d, speed_scale):
    f = lambda name: os.path.join(d, name)
    vs = load(f("__mcity__vehicle_state.csv"))
    t = np.arange(vs["t"][0], vs["t"][-1], DT)
    sw = np.interp(t, vs["t"], vs["steer_state"])
    vx_wheel = np.interp(t, vs["t"], vs["speed_x"])

    if os.path.exists(f("__ins__imu.csv")):
        imu = load(f("__ins__imu.csv"))
        r = np.interp(t, imu["t"], imu["angular_velocity.z"]); r_src = "/ins/imu"
    else:
        pose = load(f("__mcity__cav_pose.csv"))
        yaw = np.unwrap(yaw_from_quat(pose["pose.pose.orientation.x"], pose["pose.pose.orientation.y"],
                                      pose["pose.pose.orientation.z"], pose["pose.pose.orientation.w"]))
        r = np.gradient(smooth(np.interp(t, pose["t"], yaw), 5), DT); r_src = "cav_pose heading"

    if os.path.exists(f("__ins__odometry.csv")):
        od = load(f("__ins__odometry.csv"))
        v = np.interp(t, od["t"], np.hypot(od["twist.twist.linear.x"], od["twist.twist.linear.y"])); v_src = "/ins/odometry"
    else:
        v = vx_wheel * speed_scale; v_src = f"speed_x * {speed_scale}"
    r = smooth(r, 5)
    return dict(t=t, sw=sw, r=r, v=v, vx_wheel=vx_wheel, src=(r_src, v_src), name=os.path.basename(d.rstrip("/\\")))


def steady_windows(run, win=2.0, step=0.5):
    n, s = int(win / DT), int(step / DT)
    rows = []
    for i in range(0, len(run["t"]) - n, s):
        sl = slice(i, i + n)
        sw, r, v = run["sw"][sl], run["r"][sl], run["v"][sl]
        if v.mean() < 2.0 or abs(np.degrees(sw.mean())) < 5:
            continue
        if np.degrees(sw.std()) > 1.5 or v.std() > 0.2 or r.std() > 0.02:
            continue
        rows.append((sw.mean(), r.mean(), v.mean()))
    return np.array(rows)


def bicycle_yaw(p, Cf, Cr, delta, v):
    m, Iz, lf, lr = p["mass"], p["Iz"], p["lf"], p["lr"]
    vy = r = 0.0
    out = np.empty_like(delta)
    sub = 4
    h = DT / sub
    for i in range(len(delta)):
        vi = max(v[i], 1.0)
        for _ in range(sub):
            dvy = -(Cf + Cr) / (m * vi) * vy + (-vi - (Cf * lf - Cr * lr) / (m * vi)) * r + Cf / m * delta[i]
            dr = -(Cf * lf - Cr * lr) / (Iz * vi) * vy - (Cf * lf ** 2 + Cr * lr ** 2) / (Iz * vi) * r + Cf * lf / Iz * delta[i]
            vy += h * dvy; r += h * dr
        out[i] = r
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", required=True)
    ap.add_argument("--steady", nargs="+", required=True)
    ap.add_argument("--transient", nargs="*", default=[])
    ap.add_argument("--speed-scale", type=float, default=1.0, help="only used without /ins/odometry")
    ap.add_argument("--cr-norm", type=float, default=10.0,
                    help="fallback rear stiffness per unit axle load [1/rad] if no --transient")
    ap.add_argument("--update", action="store_true", help="write results into --params")
    a = ap.parse_args()
    p = json.load(open(a.params))
    m, lf, lr = p["mass"], p["lf"], p["lr"]
    L = lf + lr

    runs = [load_run(d, a.speed_scale) for d in a.steady]
    print(f"yaw-rate source: {runs[0]['src'][0]}   speed source: {runs[0]['src'][1]}")

    # wheel-speed scale check
    vw = np.concatenate([x["vx_wheel"] for x in runs]); vt = np.concatenate([x["v"] for x in runs])
    sel = vw > 2.0
    if sel.any() and runs[0]["src"][1] == "/ins/odometry":
        scale = float(np.median(vt[sel] / vw[sel]))
        print(f"speed_x scale  : true/speed_x = {scale:.3f} -> WHEEL_DIAMETER 0.673 should be ~{0.673 * scale:.3f} m "
              "(mcity_bywire_node.py WHEEL_RADIUS, preview_control sx.hpp)")
    else:
        scale = None

    # sign check (positive steer = left = positive yaw rate)
    sw = np.concatenate([x["sw"] for x in runs]); rr = np.concatenate([x["r"] for x in runs])
    flip = np.corrcoef(sw, rr)[0, 1] < 0
    if flip:
        print("NOTE: yaw rate is anti-correlated with steering -> flipping its sign (IMU z axis down?)")
        for x in runs:
            x["r"] = -x["r"]

    W = np.vstack([steady_windows(x) for x in runs if len(steady_windows(x))]) if runs else np.empty((0, 3))
    if len(W) < 6:
        raise SystemExit(f"only {len(W)} steady-state windows found; need circles at several speeds/angles")
    sw_m, r_m, v_m = W.T
    kappa, ay = r_m / v_m, v_m * r_m
    A = np.column_stack([kappa, ay, np.ones_like(kappa)])
    (a1, a2, off), *_ = np.linalg.lstsq(A, sw_m, rcond=None)
    ratio = a1 / L
    Kus = a2 / ratio
    resid = sw_m - A @ np.array([a1, a2, off])
    print(f"steady windows : {len(W)}  (v {v_m.min():.1f}-{v_m.max():.1f} m/s, |a_y| up to {np.abs(ay).max():.1f} m/s^2)")
    print(f"steering ratio : {ratio:.2f}   (sx.hpp STEERING_RATIO currently 16.0)")
    print(f"understeer Kus : {Kus:.5f} rad/(m/s^2) = {np.degrees(Kus) * G:.2f} deg/g")
    print(f"centre offset  : {np.degrees(off):.2f} deg steering wheel;  fit rms {np.degrees(resid.std()):.2f} deg")

    if a.transient:
        truns = [load_run(d, a.speed_scale) for d in a.transient]
        if flip:
            for x in truns:
                x["r"] = -x["r"]

        def res(th):
            Cf, Cr = np.exp(th)
            out = []
            for x in truns:
                sel = x["v"] > 4.0
                d = (x["sw"] - off) / ratio
                rm = smooth(bicycle_yaw(p, Cf, Cr, d, x["v"]), 5)   # same filter as the measurement
                out.append((rm - x["r"])[sel])
            kus_model = m / L * (lr / Cf - lf / Cr)
            out.append(np.array([(kus_model - Kus) / max(abs(Kus), 1e-3) * 0.5]))
            return np.concatenate(out)

        sol = least_squares(res, np.log([1.2e5, 1.2e5]), max_nfev=200)
        Cf, Cr = np.exp(sol.x)
        print(f"cornering      : Cf {Cf:.0f} N/rad, Cr {Cr:.0f} N/rad (axle), yaw-rate rms {np.sqrt(np.mean(sol.fun[:-1] ** 2)):.4f} rad/s")
    else:
        Cr = a.cr_norm * m * G * lf / L
        denom = Kus * L / m + lf / Cr
        if denom <= 0:
            raise SystemExit("Kus too negative for the fallback; record --transient sine-steer runs")
        Cf = lr / denom
        print(f"cornering      : Cf {Cf:.0f}, Cr {Cr:.0f} N/rad (axle) - FALLBACK (Cr assumed, cr_norm={a.cr_norm}); "
              "record Test 3 for a real split")
    print(f"model Kus check: {m / L * (lr / Cf - lf / Cr):.5f}")

    if a.update:
        p["steering_ratio"] = round(float(ratio), 3)
        p["Cf"], p["Cr"] = round(float(Cf)), round(float(Cr))
        meas = p.setdefault("_measured", {})
        meas["vehicle"] = {"Kus": round(float(Kus), 6), "steer_offset_deg": round(float(np.degrees(off)), 3),
                           "speed_x_scale": scale, "yaw_rate_flipped": bool(flip),
                           "cornering_from": "transient" if a.transient else "fallback"}
        json.dump(p, open(a.params, "w", newline="\n"), indent=2)
        print(f"updated {a.params}")


if __name__ == "__main__":
    main()
