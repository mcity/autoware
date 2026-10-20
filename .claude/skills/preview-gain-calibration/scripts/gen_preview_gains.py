#!/usr/bin/env python3
"""Generate preview_control gain tables (Feedback_dynamic.txt / Feedforward_dynamic.txt).

Discrete LQR preview design on the lateral error-dynamics bicycle model
(Rajamani, "Vehicle Dynamics and Control", ch. 2.5), optionally augmented with a
first-order steering lag and an N-step pure command delay. Output layout matches
what src/mcity/preview_control/src/path_follow/path_follow.cpp reads:

  Feedback_dynamic.txt   (one row per speed, v = 4..30 m/s)
    withoutdelay: idx v k_ey k_eydot k_ephi k_ephidot
    withdelay   : idx v k_ey k_eydot k_ephi k_ephidot k_delta k_d[0..4]
                  (k_d[0] multiplies the OLDEST buffered command, k_d[4] the newest)
  Feedforward_dynamic.txt
    idx v kff[0..149]    kff[j] multiplies road curvature j*dt seconds ahead

Sign convention (matches path_follow.cpp, u = tan(front wheel angle)):
    u = -K_x . x - kff . curvature,  x = [e1, e1_dot, e2, e2_dot, delta, d0..d4]
    e1 = lateral offset of vehicle from path (left +)   = -p2c.ey
    e2 = vehicle yaw - path heading            (left +) = -p2c.ephi

Usage:
    python gen_preview_gains.py --params mache.json --out <dir>            # both variants
    python gen_preview_gains.py --params mkz.json --compare <preview_control/data/gain>
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.linalg import expm, solve_discrete_are

N_DELAY_SLOTS = 5      # fixed by path_follow.cpp (delayedCmd[5])
N_PREVIEW = 150        # columns in Feedforward_dynamic.txt
SPEEDS = np.arange(4.0, 31.0, 1.0)  # 27 rows, path_follow indexes by (int)(v - 4)

DEFAULT_WEIGHTS = {
    "q_ey": 1.0, "q_eydot": 0.0, "q_ephi": 1.0, "q_ephidot": 0.0, "q_delta": 0.0, "r": 1.0,
    "speed_exp": {},
}


def continuous_model(p, v, with_lag):
    """Error-dynamics bicycle model. Cf/Cr are AXLE cornering stiffness [N/rad]."""
    m, Iz, lf, lr, Cf, Cr = p["mass"], p["Iz"], p["lf"], p["lr"], p["Cf"], p["Cr"]
    A = np.array([
        [0, 1, 0, 0],
        [0, -(Cf + Cr) / (m * v), (Cf + Cr) / m, (-Cf * lf + Cr * lr) / (m * v)],
        [0, 0, 0, 1],
        [0, -(Cf * lf - Cr * lr) / (Iz * v), (Cf * lf - Cr * lr) / Iz,
         -(Cf * lf ** 2 + Cr * lr ** 2) / (Iz * v)],
    ])
    B = np.array([[0], [Cf / m], [0], [Cf * lf / Iz]])
    # disturbance input is road curvature kappa; psi_dot_des = v * kappa
    E = v * np.array([[0], [-(Cf * lf - Cr * lr) / (m * v) - v], [0],
                      [-(Cf * lf ** 2 + Cr * lr ** 2) / (Iz * v)]])
    if not with_lag:
        return A, B, E
    tau = p["tau"]
    A5 = np.zeros((5, 5))
    A5[:4, :4] = A
    A5[:4, 4:5] = B          # actual wheel angle drives the vehicle
    A5[4, 4] = -1.0 / tau
    B5 = np.zeros((5, 1)); B5[4, 0] = 1.0 / tau
    E5 = np.zeros((5, 1)); E5[:4] = E
    return A5, B5, E5


def c2d(A, B, E, dt):
    n, nb = A.shape[0], B.shape[1] + E.shape[1]
    M = np.zeros((n + nb, n + nb))
    M[:n, :n] = A
    M[:n, n:] = np.hstack([B, E])
    Md = expm(M * dt)
    Ad, BEd = Md[:n, :n], Md[:n, n:]
    return Ad, BEd[:, :B.shape[1]], BEd[:, B.shape[1]:]


def delay_steps(p):
    nd = int(round(p["delay"] / p["dt"]))
    if nd > N_DELAY_SLOTS:
        print(f"WARNING: delay {p['delay']}s = {nd} steps > {N_DELAY_SLOTS} buffer slots; clipping",
              file=sys.stderr)
        nd = N_DELAY_SLOTS
    return nd


def weights_at(w, v):
    """Weights may be speed-scheduled: q(v) = q * (v / 10) ** speed_exp[q]."""
    exp = w.get("speed_exp", {})
    return {k: (val * (v / 10.0) ** exp.get(k, 0.0) if not isinstance(val, dict) else val)
            for k, val in w.items()}


def design(p, w, v, with_delay, n_preview=N_PREVIEW):
    """Returns (kfb row without idx/v, kff row of length n_preview)."""
    w = weights_at(w, v)
    dt = p["dt"]
    Ac, Bc, Ec = continuous_model(p, v, with_delay)
    Ad, Bd, Ed = c2d(Ac, Bc, Ec, dt)
    nx = Ad.shape[0]

    nd = delay_steps(p) if with_delay else 0
    # state z = [x, d_oldest .. d_newest]; actuator sees d_oldest; u enters d_newest
    nz = nx + nd
    Az = np.zeros((nz, nz)); Bz = np.zeros((nz, 1)); Ez = np.zeros((nz, 1))
    Az[:nx, :nx] = Ad; Ez[:nx] = Ed
    if nd > 0:
        Az[:nx, nx:nx + 1] = Bd
        for i in range(nd - 1):
            Az[nx + i, nx + i + 1] = 1.0
        Bz[nz - 1, 0] = 1.0
    else:
        Bz[:nx] = Bd

    # preview augmentation: c_0..c_{Np-1}, shift register
    Np = n_preview
    n = nz + Np
    A = np.zeros((n, n)); B = np.zeros((n, 1))
    A[:nz, :nz] = Az
    if Np > 0:
        A[:nz, nz:nz + 1] = Ez
    for j in range(Np - 1):
        A[nz + j, nz + j + 1] = 1.0
    B[:nz] = Bz

    q = [w["q_ey"], w["q_eydot"], w["q_ephi"], w["q_ephidot"]]
    if with_delay:
        q.append(w.get("q_delta", 0.0))
    Q = np.zeros((n, n))
    Q[:len(q), :len(q)] = np.diag(q)
    R = np.array([[w["r"]]])

    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A).ravel()

    kx, kc = K[:nz], K[nz:]
    if with_delay:
        kd = np.zeros(N_DELAY_SLOTS)
        if nd > 0:
            kd[N_DELAY_SLOTS - nd:] = kx[nx:]   # newest commands occupy the last slots
        kfb = np.concatenate([kx[:nx], kd])
    else:
        kfb = kx[:4]
    return kfb, kc


def closed_loop_radius(p, kfb, v, with_delay, extra_steps=0, tau_scale=1.0):
    """Spectral radius of the loop as path_follow.cpp ACTUALLY implements it:
    e1_dot is replaced by v*e2 and e2_dot by 0; the plant always has the steering lag and
    (delay + extra_steps) of pure delay, even when the withoutdelay gains are used.
    Returns (spectral radius, min damping ratio of oscillatory modes). radius >= 1 -> unstable."""
    pp = dict(p, tau=p["tau"] * tau_scale)
    Ac, Bc, Ec = continuous_model(pp, v, True)
    Ad, Bd, _ = c2d(Ac, Bc, Ec, p["dt"])
    nd = int(round(p["delay"] / p["dt"])) + extra_steps
    N = max(N_DELAY_SLOTS, nd, 1)
    n = 5 + N
    F = np.zeros(n)                       # u = F @ s, s = [e1, e1d, e2, e2d, delta, u_{k-N} .. u_{k-1}]
    F[0] = -kfb[0]
    F[2] = -(kfb[1] * v + kfb[2])         # dERate = v * hE ; hERate = 0
    if with_delay:
        F[4] = -kfb[4]
        for i in range(N_DELAY_SLOTS):
            F[5 + N - N_DELAY_SLOTS + i] = -kfb[5 + i]
    A = np.zeros((n, n))
    A[:5, :5] = Ad
    if nd == 0:
        A[:5, :] += Bd @ F[None, :]
    else:
        A[:5, 5 + N - nd] += Bd[:, 0]
    for i in range(N - 1):
        A[5 + i, 5 + i + 1] = 1.0
    A[5 + N - 1, :] = F
    lam = np.linalg.eigvals(A)
    lam = lam[np.abs(lam) > 1e-9]
    s = np.log(lam.astype(complex)) / p["dt"]
    osc = s[np.abs(s.imag) > 0.5]          # ignore real / near-real modes
    zeta = float(np.min(-osc.real / np.abs(osc))) if len(osc) else 1.0
    return float(np.max(np.abs(lam))), zeta


def stability_report(p, fb, with_delay):
    print("         nominal plant    |  min damping with robustness margins")
    print("   v   radius   damping   |  +40ms delay  +80ms delay  tau*1.5")
    ok = True
    for i in (0, 2, 6, 11, 16, 21, 26):
        v = SPEEDS[i]
        r = [closed_loop_radius(p, fb[i], v, with_delay, e, ts) for e, ts in ((0, 1), (1, 1), (2, 1), (0, 1.5))]
        z = [x[1] if x[0] < 1 else float("-inf") for x in r]
        flag = "  UNSTABLE" if r[0][0] >= 1 else ("  oscillatory" if z[0] < 0.2 else
                                                 ("  thin margin" if min(z) < 0.1 else ""))
        ok &= not flag
        zs = lambda x: "  unstable" if x == float("-inf") else f"{x:10.2f}"
        print(f"  {v:4.0f}  {r[0][0]:6.3f}  {zs(z[0])}   |  {zs(z[1])}   {zs(z[2])}   {zs(z[3])}{flag}")
    return ok


def generate(p, w, with_delay):
    fb, ff = [], []
    for v in SPEEDS:
        kfb, kff = design(p, w, v, with_delay)
        fb.append(kfb); ff.append(kff)
    return np.array(fb), np.array(ff)


def write_table(path, rows):
    with open(path, "w", newline="\n") as f:
        for i, (v, r) in enumerate(zip(SPEEDS, rows), start=1):
            f.write(f"{i} {v:.6f}  " + "  ".join(f"{x:.6f}" for x in r) + "  \n")


def read_table(path):
    return np.array([[float(x) for x in line.split()] for line in open(path) if line.strip()])


def compare(gen, ref, label):
    ref = ref[:, 2:2 + gen.shape[1]]
    print(f"\n== {label}  (rows: v=4,10,20,30)")
    for i in (0, 6, 16, 26):
        print(f"v={SPEEDS[i]:4.0f} gen: " + " ".join(f"{x:8.4f}" for x in gen[i, :10]))
        print(f"       ref: " + " ".join(f"{x:8.4f}" for x in ref[i, :10]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", required=True, help="vehicle params JSON (see mache.json)")
    ap.add_argument("--out", help="output gain folder; writes withdelay/ and withoutdelay/")
    ap.add_argument("--compare", help="existing gain folder to print side-by-side against")
    args = ap.parse_args()

    p = json.load(open(args.params))
    print(f"vehicle: {p.get('name', '?')}  delay steps: {delay_steps(p)}  tau: {p['tau']}s")

    for variant, with_delay in (("withoutdelay", False), ("withdelay", True)):
        # withdelay may carry its own weights; otherwise both variants share "weights"
        w = dict(DEFAULT_WEIGHTS, **p.get("weights", {}))
        if with_delay:
            w = dict(w, **p.get("weights_withdelay", {}))
        print(f"{variant} weights: {w}")
        fb, ff = generate(p, w, with_delay)
        print(f"closed-loop spectral radius, {variant} gains on the measured plant (lag + delay):")
        stability_report(p, fb, with_delay)
        if args.out:
            d = os.path.join(args.out, variant)
            os.makedirs(d, exist_ok=True)
            write_table(os.path.join(d, "Feedback_dynamic.txt"), fb)
            write_table(os.path.join(d, "Feedforward_dynamic.txt"), ff)
            print(f"wrote {d}")
        if args.compare:
            d = os.path.join(args.compare, variant)
            compare(fb, read_table(os.path.join(d, "Feedback_dynamic.txt")), f"{variant} feedback")
            compare(ff, read_table(os.path.join(d, "Feedforward_dynamic.txt")), f"{variant} feedforward[0:10]")


if __name__ == "__main__":
    main()
