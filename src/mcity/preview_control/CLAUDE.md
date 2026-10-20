# preview_control (Ford Mach-E)

Lateral LQR preview controller + PI speed controller, 50 Hz. On the real car it runs with Mach-E
gain tables only; the MKZ tables are kept just for CARLA.

## Launch
- Real car: `ros2 launch preview_control mache_control.launch.py [gain_variant:=withdelay|withoutdelay] [slope_file:=<path>]`
  - `gain_folder` = `data/gain/mache/<gain_variant>/` (default `withdelay`)
  - `consider_delay_lag` = true for withdelay (delay buffer + measured-steering feedback), false for withoutdelay
  - `steering_ratio` is read from `data/gain/mache/params.json` (stays in sync with the tables)
  - `preview_time` 5.0 s, `desired_time_resolution` 0.04 s (= the gain tables' time step)
  - `heading_offset` 0.0 (as stanley_control on the MachE). The MKZ's -0.03 rad caused a constant ~10 deg
    left steering bias on the bench
  - `slope_file` empty = flat road (slope 0). A slope map also changes speed control (gravity comp + pedal offsets)
- CARLA: `carla_control.launch.py` -> `data/gain/mkz/withoutdelay/` (MKZ tables, unchanged behaviour)

## Command chain on the car (launching this node alone does NOT move the car)
`/mcity/vehicle_planning` + `/mcity/cav_pose` + `/mcity/vehicle_state` -> **preview_control**
-> `/mcity/preview_control/vehicle_control` -> relay `ros2 run mcity_vigrade mcity_preview_control`
(~/mcity_demos; forwards only when the redis control-override FLAG is "False")
-> `/mcity/vehicle_control` -> `mcity_bywire_node.py` (~/mcity_mache_bywire) -> Dataspeed DBW.
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
- Current set (2026-10-20) is PROVISIONAL: template values (mass 2300, ratio 16, Cf=Cr=150k, tau 0.15,
  delay 0.2), r = 16 withoutdelay / 4 withdelay. Replace by driving + `update_from_drive.py`.
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
- `p2c.cr_vector` is never filled -> feedforward inactive; only feedback runs.
- path_process: `process_path()` downsamples the plan to one point per 0.04 s of travel (vd floored at
  1.5 m/s) up to `preview_time`; falls back to fixed 0.2 m `resampling()` if either param is <= 0.
  The closest index is stepped back ~1 m along the path (localization offset).
- Known bug, left as is: `pathProcess.init()` is called with `max_curvature`/`max_ephi` swapped
  (only affects warning thresholds).
- Speed-control constants in `sx.hpp` (`VEHICLE_WEIGHT` 1800, `WHEEL_DIAMETER` 0.673, `BRAKETABLE`) are
  still MKZ values; lateral control does not use them.
- Build: `colcon build --packages-select preview_control` (C++ changes only).
