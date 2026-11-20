# preview_control (Ford Mach-E)

Lateral LQR preview controller + PI speed controller, 50 Hz. On the real car it runs with Mach-E
gain tables only; the MKZ tables are kept just for CARLA.

## Launch
- Real car: `ros2 launch preview_control mache_control.launch.py [gain_variant:=withdelay|withoutdelay] [slope_file:=<path>]`
  - `gain_folder` = `data/gain/mache/<gain_variant>/` (default `withdelay`)
  - `consider_delay_lag` = true for withdelay (delay buffer + measured-steering feedback), false for withoutdelay
  - `steering_ratio` is read from `data/gain/mache/params.json` (stays in sync with the tables)
  - `preview_time` 5.0 s, `desired_time_resolution` 0.04 s (= the gain tables' time step)
  - `heading_offset` -0.03: the pose heading is biased. The 2026-10-26 drive showed a steady -0.035 rad ephi
    with a constant 0.6 m ey on straights. A bench test with a perfect fake pose shows a ~10 deg bias from it: expected
  - Smoothing (2026-10-26, after "brakes constantly / steering aggressive" on the 2nd drive):
    `max_speed` 6.0 caps the planned speed. Speed PI `kp` 1.0 / `ki` 0.3 (MKZ: 1.8 / 0.6).
    `coast_band` 0.6 m/s^2: lift off (regen) before braking. `throttle_rate` 0.3/s and `brake_rate` 0.5/s
    cap pedal rise. `steer_rate_limit` 3.0 rad/s. All of these default to 0 = off in the node.
  - Lift-off blend (2026-11-20, after "brakes in between / surging" at the 6 m/s cap): the MKZ throttle table's
    lowest entry is 0.15, so any request below it cut throttle 0.15 -> 0, and on the MachE 0 = regen
    (~-0.3 m/s^2). Result: a 2.4 s surge/regen limit cycle (5.75 <-> 6.27 m/s) with brake_cmd = 0.
    `lift_decel` 0.35 m/s^2 blends throttle linearly from 0.15 to 0 over that decel range (provisional,
    measured -0.27..-0.48 at 3-7 m/s). `reset_when_dbw_off` true clears the PI integral and pedal rate
    limiters while by-wire is off, so engaging after manual driving doesn't surge. Both default off in the node.
  - Steering smoothing (2026-11-20, "steering sudden, slow down when steering"): `steer_filter_tau` 0.15 s
    first-order low-pass on the steering command (tracks the measured wheel while by-wire is off, so engaging
    doesn't jump), `steer_rate_limit` 3.0 -> 2.0 rad/s, curve speed `curve_lat_acc` 1.0 -> 0.6,
    `curve_decel` 0.8 -> 0.6, `curve_min_speed` 2.5 -> 2.0. The filter lag is in the gain tables:
    params.json `tau` 0.244 = actuator 0.094 + filter 0.15. Change one, regenerate with the other.
  - 2026-11-20 (later): turns were too slow with steer_rate_limit 2.0 (path-departure stop in the sharp right
    turn), and the driver still felt sudden steering/throttle: steer_rate_limit 2.5, steer_filter_tau 0.20
    (params.json tau 0.294), feedforward_gain 1.0, throttle_rate 0.15, new `throttle_fall_rate` 0.3 (throttle
    eases off unless a brake is wanted; brake requests still cut it at once).
  - 2026-11-20 12:51, first drive with a healthy INS feed (OXTS `pub_path_rate` fix, cav_pose 50 Hz): straights
    mean |ey| 0.14 m, but curves were cut ~0.4 m to the inside (80 % of curve time, both directions) ->
    feedforward_gain 1.0 -> 0.75. Drives before 12:47 that day had a 2-10 Hz cav_pose with up to 1 s freezes,
    so judge tunings only on later drives.
  - MachE brake cmd is 0..1 = 0..100 % (by-wire sends brake_cmd*100, and any brake > 0 cuts throttle).
    `brake_gain` 0.05: brake = 0.05 * decel [m/s^2]. The MKZ BRAKETABLE starts at 0.184 (= 18 % on the MachE)
    and is used only when brake_gain = 0. set_stop/estop keep their fixed 0.22-0.235 holds.
  - Curve speed limit (2026-11-05): `curve_lat_acc` 1.0 m/s^2 caps vd at sqrt(a/curvature) (12 m radius
    -> 3.5 m/s), `curve_decel` 0.8 m/s^2 ramps down before the turn, `curve_min_speed` 2.5 m/s floor.
    Curvature is from path x/y (heading change over +-2 m), in `compute_curve_speed()`; the cap is applied
    last in `get_desired_velocity()`. `curve_lat_acc` 0 = off. Nov 2 drives took turns at up to 1.8 m/s^2.
  - `slope_file` empty = flat road (slope 0). A slope map also changes speed control (gravity comp + pedal offsets)
- CARLA: `carla_control.launch.py` -> `data/gain/mkz/withoutdelay/` (MKZ tables, unchanged behaviour)

## Command chain on the car (launching this node alone does NOT move the car)
`/mcity/vehicle_planning` + `/mcity/cav_pose` + `/mcity/vehicle_state` -> **preview_control**
-> `/mcity/preview_control/vehicle_control` -> relay `ros2 run mcity_vigrade mcity_preview_control`
(~/mcity_demos; forwards only when the redis control-override FLAG is "False")
-> `/mcity/vehicle_control` -> `mcity_mache_bywire` (src/mcity/mcity_mache_bywire) -> Dataspeed DBW.
stanley_control and pursuit_control publish to the same `/mcity/preview_control/vehicle_control`
topic: run only one controller at a time.

## Library shadowing (important)
~/.bashrc sources ~/mcity_demos, whose `autoware_preview_control` installs another
`libpreview_control_component.so` that comes first on `LD_LIBRARY_PATH`. A plain `ros2 run preview_control`
loads THAT code. `mache_control.launch.py` prepends this package's lib dir via `additional_env`.
Quick check: our build logs `Loaded <N> slope points from ...`; the other one logs `Suceessfully loaded slope data.`

## Gain tables
- `data/gain/mache/{withdelay,withoutdelay}/{Feedback,Feedforward}_dynamic.txt` + `params.json`
  (vehicle params, LQR weights, `_measured`, `_history`). Generated, never hand-edited.
- Current set is PROVISIONAL: template vehicle values (mass 2300, ratio 16, Cf=Cr=150k, tau 0.15, delay 0.2).
  2026-10-26 retune after the first drive (ey drifted to 1.6 m -> path-departure stops): withdelay r=1 and
  q_ey x4 (k_ey ~ the old MKZ x1.4 setup). Eased to q_ey x2 after the 2nd drive (steering too aggressive). Damping >= 0.33 up to 15 m/s, thin above (not for highway
  speed yet). withoutdelay r=32. `update_from_drive.py --vmax` defaults to 15 m/s.
  2026-11-20: steering weaved in turns (S-overshoot, |ey| 1.3-1.7 m). Drive fit: delay 0.03 s, tau 0.094 s, K 1.00,
  so the old tables (designed for delay 0.2 / tau 0.15) had damping ~0.20 on the real car: they treated 5 already-applied
  commands as still pending (k_d spread over all buffer slots). Regenerated with delay 0.04 (1 step), tau 0.094,
  withdelay r=2: damping 0.76 at 6 m/s, k_ey 71 % of before. Same day, manual baseline drive: steering ratio
  16 -> 14.43 (11 steady windows, 2.4-9.1 m/s; Kus 1.4 deg/g). The tables are in front-wheel angle, so only
  params.json changed (old 16 over-commanded the wheel by ~11 %). Cf/Cr still template (fit 39k rejected).
  `update_from_drive.py` rejects delay < 0.04 and judges stability on params.json, so it picked r=0.5 against the
  old 0.2 s model: check the installed table on the measured plant (`closed_loop_radius`) before trusting its r.
- Regenerate / calibrate with the skill in `~/autoware/.claude/skills/preview-gain-calibration/` (SKILL.md):
  ```bash
  S=~/autoware/.claude/skills/preview-gain-calibration/scripts; source ~/autoware/install/setup.bash
  $S/record_drive.sh                                  # drive, Ctrl-C
  python3 $S/update_from_drive.py ~/mache_calib/drives/bags/<drive>   # fit, gate, retune, install (+backup)
  ```
  Gains load at start-up only: relaunch after an update. No rebuild needed for table/params changes.
- Measured so far (Aug 2026 by-wire drives, DBW report): steering tracks 1:1 (K = 1.00), ECU delay 0.05 s,
  tau 0.014 s. Not applied, because it misses controller->by-wire latency. Hence no `* 1.4` in `publishCmd()`.

## Code facts
- Control law (path_follow.cpp): `tan(delta) = -K*[ey, v*ephi, ephi, 0, delta_meas, cmd buffer] - kff*curvature`;
  `dERate = v*hE`, `hERate = 0`; 5-slot command buffer updated every 0.04 s.
- Feedforward (curvature preview) is active since 2026-11-20 when `feedforward_gain` > 0 (mache launch: 0.75):
  `path_process` computes signed path curvature (left +, 2 m chord window) and fills `p2c.cr_vector` with it
  every `max(v,1)*0.04` m ahead of the closest point; `path_follow` sums all 150 kff columns (was 50,
  which over-steered ~15 %). Replay check of the 2026-11-20 drive: FF turns the same way, ~1 s earlier.
- path_process: `process_path()` downsamples the plan to one point per 0.04 s of travel (vd floored at
  1.5 m/s) up to `preview_time`; falls back to fixed 0.2 m `resampling()` if either param is <= 0.
  The closest index is stepped back ~1 m along the path (localization offset).
- Known bug, left as is: `pathProcess.init()` is called with `max_curvature`/`max_ephi` swapped
  (only affects warning thresholds).
- Speed-control constants in `sx.hpp` (`VEHICLE_WEIGHT` 1800, `WHEEL_DIAMETER` 0.673, `BRAKETABLE`) are
  still MKZ values; lateral control does not use them.
- Build: `colcon build --packages-select preview_control` (C++ changes only).
