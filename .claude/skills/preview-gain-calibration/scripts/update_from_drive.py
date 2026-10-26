#!/usr/bin/env python3
"""Update the MachE preview_control gain tables from ordinary drives (record_drive.sh bags).

    source ~/autoware/install/setup.bash
    python3 update_from_drive.py ~/mache_calib/drives/bags/<drive> [...] [--only-new] [--no-install]

1. exports each new bag to <store>/csv/<name>               (bag_to_csv.py)
2. fits steering delay/lag/K on all by-wire segments         (fit_steering.py)
   fits steering ratio, Kus, Cf/Cr on steady + transient bits (fit_vehicle.py)
   using every drive in the store (or only the given ones with --only-new)
3. accepts each fitted group only if it is inside the plausible range (SKILL.md table),
   otherwise keeps the current value
4. per variant, picks the smallest r in R_LADDER whose gains pass the stability check
   (nominal damping >= 0.3 and stable with +40 ms delay, all speeds <= --vmax, default 15 m/s)
5. writes data/gain/mache/{withdelay,withoutdelay,params.json}; the old set is backed up
   to <store>/backups/<timestamp>/. preview_control loads gains at start-up: relaunch it.

Bags without /mcity/vehicle_state are reconstructed from /vehicle/steering/report; their steering
delay/tau are reported but not applied (no controller->by-wire latency in that command).
Drives with by-wire steering give the actuator fit; curves at steady speed give the ratio;
lane changes / S-curves above 4 m/s give Cf/Cr. Manual-only drives still give ratio + Cf/Cr.
"""
import argparse
import copy
import datetime
import json
import os
import shutil
import subprocess
import sys

os.environ.setdefault("OMP_NUM_THREADS", "1")   # small problems; BLAS threads only thrash
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_preview_gains as gen  # noqa: E402

GAIN_DIR = os.path.expanduser("~/autoware/src/mcity/preview_control/data/gain/mache")
R_LADDER = (0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0)
RANGES = {          # plausible MachE values (SKILL.md); outside -> keep the current value
    "delay": (0.04, 0.20), "tau": (0.03, 0.25),
    "steering_ratio": (13.0, 18.0), "Cf": (80e3, 250e3), "Cr": (80e3, 250e3),
}


def run(cmd, log):
    print("$ " + " ".join(cmd))
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")   # small problems; threads only thrash
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    out = p.stdout + p.stderr
    print(out.rstrip())
    log.write("$ " + " ".join(cmd) + "\n" + out + "\n")
    return p.returncode == 0


def in_range(p, keys):
    bad = [k for k in keys if not RANGES[k][0] <= p[k] <= RANGES[k][1]]
    for k in bad:
        print(f"REJECT {k} = {p[k]:g} outside {RANGES[k]}")
    return not bad


def check(p, fb, with_delay, vmax):
    """-> (min nominal damping, stable with +40 ms) over speeds <= vmax."""
    zmin, robust = 1.0, True
    for i, v in enumerate(gen.SPEEDS):
        if v > vmax:
            break
        rad, z = gen.closed_loop_radius(p, fb[i], v, with_delay, 0, 1)
        zmin = min(zmin, z if rad < 1 else -1.0)
        robust &= gen.closed_loop_radius(p, fb[i], v, with_delay, 1, 1)[0] < 1
    return zmin, robust


def tune(p, variant, vmax):
    """Smallest r (sharpest tracking) that passes; falls back to the best-damped one."""
    key = "weights_withdelay" if variant == "withdelay" else "weights"
    with_delay = variant == "withdelay"
    best = None
    for r in R_LADDER:
        q = copy.deepcopy(p)
        q[key]["r"] = r
        w = dict(gen.DEFAULT_WEIGHTS, **q.get("weights", {}))
        if with_delay:
            w = dict(w, **q.get("weights_withdelay", {}))
        fb, ff = gen.generate(q, w, with_delay)
        zmin, robust = check(q, fb, with_delay, vmax)
        print(f"  {variant:12s} r={r:<5g} min damping {zmin:5.2f}  +40ms {'ok' if robust else 'UNSTABLE'}")
        if zmin >= 0.3 and robust:
            return r, fb, ff, True
        if best is None or zmin > best[3]:
            best = (r, fb, ff, zmin)
    return best[0], best[1], best[2], False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="*", help="new bag dirs (already-exported ones are skipped)")
    ap.add_argument("--store", default=os.environ.get("MACHE_CALIB_STORE", "~/mache_calib/drives"))
    ap.add_argument("--only-new", action="store_true", help="fit only the given bags, not the whole store")
    ap.add_argument("--vmax", type=float, default=15.0,
                    help="highest speed the stability check must pass [m/s]; raise with the test speed "
                         "(25 forces much softer lateral gains)")
    ap.add_argument("--no-install", action="store_true", help="write results to the store only")
    a = ap.parse_args()

    store = os.path.expanduser(a.store)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    work = os.path.join(store, "runs", stamp)
    os.makedirs(work)
    log = open(os.path.join(work, "log.txt"), "w")

    # 1. export
    new = []
    for b in a.bags:
        d = os.path.join(store, "csv", os.path.basename(os.path.normpath(b)))
        if not os.path.isdir(b):
            raise SystemExit(f"no such bag: {b} (was record_drive.sh running?)")
        if not os.path.exists(os.path.join(d, "__mcity__vehicle_state.csv")) and not run([sys.executable, f"{HERE}/bag_to_csv.py", b, "--out", d], log):
            raise SystemExit(f"bag export failed for {b} (is the workspace sourced?)")
        new.append(d)
    csv_root = os.path.join(store, "csv")
    csvs = new if a.only_new else sorted(os.path.join(csv_root, x) for x in os.listdir(csv_root)
                                         if os.path.exists(os.path.join(csv_root, x, "__mcity__vehicle_state.csv")))
    if not csvs:
        raise SystemExit("no drives to fit")
    print(f"fitting on {len(csvs)} drive(s)")

    # 2./3. fit; each fitter writes into its own scratch copy
    cur = json.load(open(os.path.join(GAIN_DIR, "params.json")))
    new_p = copy.deepcopy(cur)
    accepted = []

    steer = os.path.join(work, "steer.json")
    json.dump(cur, open(steer, "w"), indent=2)
    full_loop = all(not os.path.exists(os.path.join(c, "DERIVED_FROM_DBW_REPORT")) for c in csvs)
    if run([sys.executable, f"{HERE}/fit_steering.py", *csvs, "--params", steer, "--update",
            "--plot", os.path.join(work, "steer_fit.png")], log):
        s = json.load(open(steer))
        if not full_loop:
            print("steering delay/tau NOT applied: some drives lack /mcity/vehicle_control, so their command is the "
                  "DBW report's (misses controller->by-wire latency). Record with record_drive.sh, or --only-new.")
        elif in_range(s, ["delay", "tau"]):
            new_p["tau"], new_p["delay"] = s["tau"], s["delay"]
            accepted.append("steering")
        new_p.setdefault("_measured", {})["steering"] = s["_measured"]["steering"]

    veh = os.path.join(work, "vehicle.json")
    json.dump(new_p, open(veh, "w"), indent=2)   # bicycle fit uses the accepted tau
    if run([sys.executable, f"{HERE}/fit_vehicle.py", "--params", veh, "--steady", *csvs,
            "--transient", *csvs, "--update"], log):
        v = json.load(open(veh))
        if in_range(v, ["steering_ratio"]):
            new_p["steering_ratio"] = v["steering_ratio"]
            accepted.append("steering_ratio")
            if v["_measured"]["vehicle"]["cornering_from"] == "transient" and in_range(v, ["Cf", "Cr"]):
                new_p["Cf"], new_p["Cr"] = v["Cf"], v["Cr"]
                accepted.append("cornering")
        new_p.setdefault("_measured", {})["vehicle"] = v["_measured"]["vehicle"]

    # 4. regenerate + stability-tune each variant
    print("\nstability tuning (smallest passing r):")
    tables, passed = {}, True
    for variant, key in (("withoutdelay", "weights"), ("withdelay", "weights_withdelay")):
        r, fb, ff, ok = tune(new_p, variant, a.vmax)
        new_p[key]["r"] = r
        tables[variant] = (fb, ff)
        passed &= ok
        if not ok:
            print(f"WARNING: no r passes for {variant} up to {a.vmax} m/s; using r={r} (best damped)")

    new_p["name"] = "Ford Mustang Mach-E (Mcity) - fitted from drives" if accepted else cur["name"]
    new_p.setdefault("_history", []).append({
        "time": stamp, "drives": [os.path.basename(c) for c in csvs], "accepted": accepted,
        "values": {k: new_p[k] for k in ("tau", "delay", "steering_ratio", "Cf", "Cr")},
        "r": {"withoutdelay": new_p["weights"]["r"], "withdelay": new_p["weights_withdelay"]["r"]}})

    out = os.path.join(work, "gain")
    for variant, (fb, ff) in tables.items():
        os.makedirs(os.path.join(out, variant), exist_ok=True)
        gen.write_table(os.path.join(out, variant, "Feedback_dynamic.txt"), fb)
        gen.write_table(os.path.join(out, variant, "Feedforward_dynamic.txt"), ff)
    json.dump(new_p, open(os.path.join(out, "params.json"), "w", newline="\n"), indent=2)

    print("\n          current     new")
    for k in ("delay", "tau", "steering_ratio", "Cf", "Cr"):
        print(f"{k:14s} {cur[k]:<10g}  {new_p[k]:<10g}")
    print(f"{'r withoutdelay':14s} {cur['weights']['r']:<10g}  {new_p['weights']['r']:<10g}")
    print(f"{'r withdelay':14s} {cur['weights_withdelay']['r']:<10g}  {new_p['weights_withdelay']['r']:<10g}")
    print(f"accepted: {', '.join(accepted) or 'nothing (tables regenerated from current values)'}")

    # 5. install
    if a.no_install:
        print(f"\nresults in {out} (not installed)")
        return
    backup = os.path.join(store, "backups", stamp)
    shutil.copytree(GAIN_DIR, backup)
    for variant in tables:
        for f in ("Feedback_dynamic.txt", "Feedforward_dynamic.txt"):
            shutil.copy(os.path.join(out, variant, f), os.path.join(GAIN_DIR, variant, f))
    shutil.copy(os.path.join(out, "params.json"), os.path.join(GAIN_DIR, "params.json"))
    print(f"\ninstalled into {GAIN_DIR}  (previous set: {backup})")
    print("relaunch preview_control (mache_control.launch.py) to load the new gains"
          + ("" if passed else "  -- stability check NOT passed, drive carefully / stay below the failing speed"))


if __name__ == "__main__":
    main()
