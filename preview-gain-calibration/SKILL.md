---
name: preview-gain-calibration
description: Calibrate the preview_control lateral gain tables (data/gain/*/Feedback_dynamic.txt, Feedforward_dynamic.txt) for a new vehicle, e.g. the Ford Mach-E, from short system-identification drives. Use when asked to generate, regenerate, re-tune or validate preview_control gains, to run steering step/chirp/sine or steady-circle tests, to fit steering lag/delay, steering ratio, understeer or cornering stiffness, or to port preview_control from the MKZ to the MachE.
---

# Preview-control gain calibration (MKZ → MachE)

The gain tables in `src/mcity/preview_control/data/gain/` are **not recorded from driving**.
They are computed offline by discrete LQR preview control on a lateral bicycle model.
To port them to a new car you measure ~6 vehicle parameters with three short tests
on an open pad (≈1–1.5 h total), fit them, and regenerate the tables with the scripts here.

The slope files (`data/slope/DIT.txt`, `VOICES.txt`) are **road-grade maps of the track**
(`(x, y): grade`), not vehicle data — they need no change. preview_control never even loads them.

## What is already established (don't re-derive)

**Table layout** (read by `src/mcity/preview_control/src/path_follow/path_follow.cpp`):

| File | Row = speed 4…30 m/s (27 rows) |
|---|---|
| `withoutdelay/Feedback_dynamic.txt` | `idx v k_ey k_eydot k_ephi k_ephidot` |
| `withdelay/Feedback_dynamic.txt` | `… k_delta_actual k_d[0..4]` (k_d[0] × OLDEST buffered cmd) |
| `Feedforward_dynamic.txt` (both) | `idx v kff[0..149]`, kff[j] × road curvature j·0.04 s ahead |

Control law: `tan(δ_front) = −K·[e1, v·e2, e2, 0, δ_meas, cmd buffer] − kff·curvature`
with `e1 = −p2c.ey` (left of path +), `e2 = −p2c.ephi`, dt = 0.04 s, 5-slot command-delay buffer.

**The generator is validated**: `scripts/gen_preview_gains.py` with `scripts/mkz.json`
reproduces the original MKZ tables — withdelay feedback within ~3 %, feedforward within a few %,
withoutdelay feedback within ~13 % (only feedback was fitted; feedforward matching independently
confirms the model structure: dt 0.04, time-based preview, 5-step/0.2 s delay, steering lag τ≈0.15 s).
The back-fitted LQR weights (speed-scheduled) live in `mkz.json` and are copied into `mache.json`
so the MachE tables keep the MKZ tuning "feel" on the new plant.

**Code facts that matter for the MachE** (verify they still hold before acting on them):

1. Tables: `data/gain/mache/{withdelay,withoutdelay}/` + `params.json` (MachE, used on the car by
   `launch/mache_control.launch.py`); `data/gain/mkz/` (MKZ originals, used only by `carla_control.launch.py`).
   `consider_delay_lag` and `steering_ratio` are ROS parameters; `mache_control.launch.py` takes
   `gain_variant:=withdelay|withoutdelay` (default withdelay → consider_delay_lag true) and reads
   `steering_ratio` from `params.json`, so regenerating tables keeps the ratio in sync.
   withoutdelay gains go unstable at speed once a 0.2 s delay + lag is present; withdelay stays damped.
2. `p2c.cr_vector` is never filled (`path_process.cpp`), so **feedforward is currently inactive**;
   only feedback runs. If someone fills it later, sample curvature every `v·0.04` m ahead.
3. `path_follow` uses `dERate = v·hE` and `hERate = 0` (not true derivatives). The stability check
   in `gen_preview_gains.py` models exactly this implemented law.
4. The old `* 1.4` steering hack in `publishCmd()` is gone. August 2026 by-wire drives
   (DBW report cmd vs angle) gave K = 1.00, delay 0.05 s, τ 0.014 s at the ECU, so 1:1 is right.
   That delay excludes the controller → by-wire node latency, so it is not applied. Steering ratio
   must be the measured one, because gains are in front-wheel angle.
7. `~/.bashrc` sources `~/mcity_demos`, whose `autoware_preview_control` installs another
   `libpreview_control_component.so` that comes first on `LD_LIBRARY_PATH`: plain `ros2 run
   preview_control` loads *that* code. `mache_control.launch.py` prepends this package's lib dir.
5. `mcity_mache_bywire/mcity_bywire_node.py`: `VehicleState.yaw_rate`, `acc_y`, `speed_y`, `x/y`
   are **never filled** (so its lateral-accel safety rule 6 never fires). Yaw rate must come from
   `/ins/imu`, true speed from `/ins/odometry`. `speed_x` = wheel speed × MKZ wheel radius
   (0.673 m dia.) — likely a few % low on the MachE; `fit_vehicle.py` reports the scale.
6. By-wire steering rate limit is `cmd_rate = 300 deg/s`; steering_cmd/steer_state are
   steering-wheel radians.

## Files

```
scripts/
  steer_excitation.py   ROS 2 node: speed hold + step/chirp/sine steering via /mcity/vehicle_control
  bag_to_csv.py         rosbag2 -> one CSV per topic (needs the workspace sourced)
  fit_steering.py       steering delay, lag τ, tracking gain K   (Test 1)
  fit_vehicle.py        steering ratio, understeer Kus, Cf/Cr, speed_x scale   (Tests 2 + 3)
  gen_preview_gains.py  writes withdelay/ + withoutdelay/ tables, prints stability/robustness report
  fit_weights.py        back-fits Q/R to an existing table (already done for MKZ; rarely needed)
  mkz.json              MKZ params + fitted weights (reference)
  mache.json            MachE TEMPLATE - copy it, fill it, never commit placeholder values as final
  record_drive.sh       record a calibration bag while driving around (manual or by-wire)
  update_from_drive.py  bags -> export -> fits -> range gate -> stability-tuned tables -> install (with backup)
```

Python deps: `numpy scipy` (+ `matplotlib` for `--plot`). On the car: `sudo apt install python3-scipy python3-matplotlib`
or `pip install --user numpy scipy matplotlib`.

## Drive-around mode (default when the user is in the car)

No dedicated pad tests: record ordinary drives and update after each one. Gains load at start-up,
so they never change mid-drive. Relaunch preview_control afterwards.
```bash
S=~/autoware/preview-gain-calibration/scripts; source ~/autoware/install/setup.bash
$S/record_drive.sh                         # drive, Ctrl-C  -> ~/mache_calib/drives/bags/drive_<time>
python3 $S/update_from_drive.py ~/mache_calib/drives/bags/drive_<time>      # fits on ALL stored drives
ros2 launch preview_control mache_control.launch.py                          # loads the new tables
```
What each kind of driving feeds:
- **steering delay/τ**: by-wire driving with any controller (needs `/mcity/vehicle_control`);
  ≥ 10 s of segments where the command moves ≥ 5°
- **steering ratio**: steady curves/roundabouts (≥ 6 windows of 2 s at constant speed and steering, |sw| > 5°),
  at several speeds
- **Cf/Cr**: lane changes / S-curves above 4 m/s at steady speed (an applied value also needs an accepted ratio)

Each group is applied only if it falls in the sanity table below; otherwise the old value stays. Then
each variant gets the smallest `r` in 0.5…32 that passes (nominal damping ≥ 0.3 and stable with
+40 ms, up to `--vmax` 25 m/s). Results, plots and logs go to `~/mache_calib/drives/runs/<time>/`,
the previous set to `backups/<time>/`, and `_history` in params.json. Use `--no-install` to preview
and `--only-new` to ignore older drives (e.g. after a DBW firmware change). Read `steer_fit.png`.

## Workflow (dedicated pad tests: more accurate, especially Cf/Cr)

Work in a session directory, e.g. `CAL=~/mache_calib/$(date +%F)`; `S=~/autoware/.claude/skills/preview-gain-calibration/scripts`.
Copy the template: `cp $S/mache.json $CAL/mache.json`. Source ROS first: `source ~/autoware/install/setup.bash`.

### Step 0 — static (no driving)
Ask the user for, and write into `$CAL/mache.json`:
- variant (RWD/AWD, battery), tyre size and pressure → note in `_notes`
- `mass` as tested (corner scales best; else curb + people + equipment)
- `lf`, `lr` from front/rear axle loads: `lf = L·Fz_rear/Fz_total`, `lr = L − lf` (L ≈ 2.984 m, confirm)
- `Iz` ≈ `mass·lf·lr` if nothing better
- full-lock `steer_state` left/right (`ros2 topic echo /mcity/vehicle_state --field steer_state`)

Before driving: `ros2 topic hz /mcity/vehicle_state` (want ≥ 50 Hz), `ros2 topic hz /ins/imu`,
`ros2 topic list | grep ins`. Nothing else may publish `/mcity/vehicle_control`
(`ros2 topic info /mcity/vehicle_control` → only the excitation node during tests).

Record **every** run in its own bag:
```bash
ros2 bag record -o $CAL/bags/<run_name> /mcity/vehicle_state /mcity/vehicle_control /mcity/cav_pose \
  /ins/imu /ins/odometry /vehicle/steering/report /vehicle/steering/cmd /vehicle/wheel_speeds
```

Safety for all tests: closed pad, safety driver ready to take over, |a_y| < ~4 m/s². The excitation
node stops commanding if by-wire drops or any override is reported; Ctrl-C also stops it.

### Step 1 — Test 1: steering actuator (≈15 min, by-wire)
```bash
python3 $S/steer_excitation.py --pattern step  --amps 30 60 90 --speed 4      # bag: step_4mps
python3 $S/steer_excitation.py --pattern chirp --amps 30 --f0 0.2 --f1 2 --duration 30 --speed 4   # bag: chirp_4mps
```
Repeat each 2×. Optionally one step run at 8 m/s with `--amps 20 40`.

### Step 2 — Test 2: steady circles (≈30 min, human or by-wire)
Constant steering-wheel angle 30°, 60°, 90°; speeds 3, 5, 8, 10, 12 m/s; skip any combo above
~4 m/s² (a_y ≈ v²·(sw/ratio)/L). Both directions. Hold ≥ 3 s steady after settling.
One bag per angle/direction is fine (e.g. `circle_60_left` covering all speeds).

### Step 3 — Test 3: sine steer at speed (≈15 min, recommended)
```bash
python3 $S/steer_excitation.py --pattern sine --amps 20 --freqs 0.3 0.6 1.0 1.5 --cycles 5 --speed 10   # bag: sine_10
python3 $S/steer_excitation.py --pattern sine --amps 15 --freqs 0.3 0.6 1.0 1.5 --cycles 5 --speed 15   # bag: sine_15
```
Without Test 3, Cf/Cr fall back to an assumed rear stiffness (flagged as FALLBACK in the output).

### Step 4 — export and fit
```bash
for b in $CAL/bags/*/; do python3 $S/bag_to_csv.py $b --out $CAL/csv/$(basename $b); done
python3 $S/fit_steering.py $CAL/csv/step_* $CAL/csv/chirp_* --params $CAL/mache.json --update --plot $CAL/steer_fit.png
python3 $S/fit_vehicle.py  --params $CAL/mache.json --steady $CAL/csv/circle_* --transient $CAL/csv/sine_* --update
```
Look at `steer_fit.png` (Read it) — model must overlay the measurement. Sanity ranges for a MachE:

| Quantity | Plausible | If outside |
|---|---|---|
| delay | 0.04–0.20 s | > 0.2 s exceeds the 5-slot buffer: report it; consider bridge timer/DDS latency |
| τ | 0.03–0.25 s | check rate limit hits (big steps) — try `--fit-rate` |
| K (tracking) | ~1.0 | ≠ 1: the by-wire angle loop does not track; raise with user (fix DBW vs a documented 1/K) |
| steering ratio | 13–18 | check yaw-rate sign/units, steady windows count |
| Kus | 0.5–4 deg/g | negative → check data/sign; very large → tyre pressure / slip |
| Cf, Cr (axle) | 80k–250k N/rad each | refit with more transient data |
| speed_x scale | 1.00–1.10 | report suggested WHEEL_DIAMETER |

Each fit appends its raw numbers under `_measured` in the params file.

### Step 5 — generate and review
```bash
python3 $S/gen_preview_gains.py --params $CAL/mache.json --out $CAL/gain
python3 $S/gen_preview_gains.py --params $CAL/mache.json --compare ~/autoware/src/mcity/preview_control/data/gain   # vs MKZ
```
Read the stability report: nominal damping should be ≥ 0.3 at all speeds and the +40 ms column
must not say `unstable` for the variant that will run. If it does, de-tune: multiply `r` in the
relevant weights block by 2–4 (softer) and regenerate; to sharpen tracking, divide `r` instead.
Keep MKZ files untouched — install MachE tables side by side:
```bash
mkdir -p ~/autoware/src/mcity/preview_control/data/gain/mache && cp -r $CAL/gain/* $_/
cp $CAL/mache.json ~/autoware/src/mcity/preview_control/data/gain/mache/params.json
```

### Step 6 — integrate (done 2026-10-20; nothing to do unless the code changes)
`mache_control.launch.py` already loads `data/gain/mache/<gain_variant>/` with `consider_delay_lag`
and `steering_ratio` from params.json, so installing new tables + params.json is enough.
Optional: WHEEL_DIAMETER in `mcity_bywire_node.py` from the speed scale.
Rebuild only after C++ changes: `colcon build --packages-select preview_control`.

### Step 7 — validation drive
Progressive: 3 → 5 → 8 → 12 → 15 m/s on a known route, safety driver ready. Record the same topics
plus `/mcity/vehicle_planning`. Pass criteria: |ey| < 0.3 m on straights and < 0.5 m in curves,
no steering oscillation (look at steer_state for a sustained 0.5–2 Hz wobble), no estop from
path departure. If oscillating at speed: increase `r` (softer) for that variant or check delay;
if sluggish/cutting corners: decrease `r`, and consider wiring the feedforward (`cr_vector`).

## Tuning notes
- Weights are `q(v) = q · (v/10)^speed_exp[q]`, R = `r`. Uniformly scaling all q's by c equals scaling r by 1/c.
- `fit_weights.py --params X.json --ref <gain dir> [--fit-tau]` recovers weights from any existing
  table set; it is how `mkz.json` was produced (do not rerun unless retargeting a different reference).
- `--compare` prints generated vs reference rows for v = 4, 10, 20, 30 m/s.
