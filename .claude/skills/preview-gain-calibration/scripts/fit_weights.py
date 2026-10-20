#!/usr/bin/env python3
"""Back-fit LQR weights (Q, R) so gen_preview_gains.py reproduces an existing gain table.

Use this with the MKZ parameters (mkz.json) and the MKZ tables to recover the tuning
"feel" the original tables had, then reuse those weights with the MachE parameters.
Only feedback gains are fitted (they do not depend on preview length, so this is fast);
the feedforward gains follow from the same weights.

    python fit_weights.py --params mkz.json --ref <preview_control/data/gain> [--fit-tau]

Prints a "weights" / "weights_withdelay" JSON block to paste into the vehicle params file.
"""
import argparse
import json
import os

import numpy as np
from scipy.optimize import least_squares

from gen_preview_gains import SPEEDS, design, read_table

Q_KEYS = ["q_ey", "q_eydot", "q_ephi", "q_ephidot"]
EXP_KEYS = ["q_ey", "q_eydot", "q_ephi", "q_ephidot"]


def unpack(x, with_delay):
    w = {"r": 1.0, "speed_exp": {}}
    i = 0
    for k in Q_KEYS:
        w[k] = float(np.exp(x[i])); i += 1
    for k in EXP_KEYS:
        w["speed_exp"][k] = float(x[i]); i += 1
    w["q_delta"] = float(np.exp(x[i])) if with_delay else 0.0
    i += 1 if with_delay else 0
    return w, x[i:]


def fit(p, ref, with_delay, fit_tau):
    ncol = 10 if with_delay else 4
    ref = ref[:, 2:2 + ncol]
    # skip columns that are ~0 in the reference (e.g. unused delay slots)
    mask = np.abs(ref).max(axis=0) > 1e-6

    def resid(x):
        w, rest = unpack(x, with_delay)
        pp = dict(p)
        if fit_tau and with_delay:
            pp["tau"] = float(np.exp(rest[0]))
        r = []
        for i, v in enumerate(SPEEDS):
            try:
                kfb, _ = design(pp, w, v, with_delay, n_preview=0)
            except np.linalg.LinAlgError:
                return np.full(int(mask.sum()) * len(SPEEDS), 10.0)
            g = np.abs(kfb[:ncol][mask]) + 1e-9
            r.append(np.log(g) - np.log(np.abs(ref[i][mask]) + 1e-9))
        return np.concatenate(r)

    x0 = [np.log(0.01), np.log(1e-4), np.log(0.1), np.log(1e-4)] + [0.0] * len(EXP_KEYS)
    if with_delay:
        x0.append(np.log(1e-3))
        if fit_tau:
            x0.append(np.log(p["tau"]))
    # bounds: log-weights in [1e-10, 1e4], speed exponents in [-4, 4], tau in [0.02, 0.5] s
    lb = [np.log(1e-10)] * len(Q_KEYS) + [-4.0] * len(EXP_KEYS)
    ub = [np.log(1e4)] * len(Q_KEYS) + [4.0] * len(EXP_KEYS)
    if with_delay:
        lb.append(np.log(1e-10)); ub.append(np.log(1e4))
        if fit_tau:
            lb.append(np.log(0.02)); ub.append(np.log(0.5))
    sol = least_squares(resid, np.array(x0), bounds=(lb, ub), method="trf", x_scale="jac", max_nfev=400)
    w, rest = unpack(sol.x, with_delay)
    tau = float(np.exp(rest[0])) if (fit_tau and with_delay) else None
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    return w, tau, rms


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", required=True)
    ap.add_argument("--ref", required=True, help="gain folder containing withdelay/ and withoutdelay/")
    ap.add_argument("--fit-tau", action="store_true", help="also fit the steering lag tau (withdelay)")
    args = ap.parse_args()
    p = json.load(open(args.params))

    out = {}
    for variant, with_delay, key in (("withoutdelay", False, "weights"),
                                      ("withdelay", True, "weights_withdelay")):
        ref = read_table(os.path.join(args.ref, variant, "Feedback_dynamic.txt"))
        w, tau, rms = fit(p, ref, with_delay, args.fit_tau)
        print(f"{variant}: rms log-error {rms:.3f} (~{100 * (np.exp(rms) - 1):.0f}% typical gain mismatch)"
              + (f", fitted tau {tau:.3f}s" if tau else ""))
        out[key] = {k: (float(f"{v:.6g}") if not isinstance(v, dict) else {kk: round(vv, 4) for kk, vv in v.items()})
                    for k, v in w.items()}
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
