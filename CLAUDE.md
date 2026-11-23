# Mcity Autoware (Ford Mach-E)

Mcity's fork of Autoware Universe (Oct 2023 base). Mcity code lives in `src/mcity/`. The real car is a
Ford Mach-E driven through a Dataspeed DBW kit.

**`main` targets ROS 2 Jazzy on Ubuntu 24.04** (gcc 13, Python 3.12). Since 2026-11-23 the Jazzy port (formerly branch
`autoware_ubuntu_24`) is `main` on GitHub; the old Humble / Ubuntu 22.04 `main` is now branch `autoware_ubuntu_22`.
Jazzy port notes:
- Use the `.hpp` headers for cv_bridge, tf2_geometry_msgs, tf2_eigen and image_geometry. Jazzy removed the `.h` versions.
- `autoware_cmake` keeps `-Werror` but makes deprecated-declarations and gcc 13
  dangling-reference/maybe-uninitialized warnings non-fatal, and defines `ROS_DISTRO_JAZZY`.

## Fresh-machine setup

Clone from **`https://github.com/mcity/autoware.git`** (remote `origin`). The README's
`michigan-traffic-lab/autoware` URL (remote `mtl`) is far behind and has none of the Mach-E work.

Prerequisites: Ubuntu 24.04 with ROS 2 Jazzy Desktop installed in `/opt/ros/jazzy`. The user has to run the `sudo` steps.

```bash
sudo apt-get install -y libhiredis-dev libgeographiclib-dev ccache python3-rosdep \
  python3-colcon-common-extensions ros-jazzy-rmw-cyclonedds-cpp
git clone https://github.com/mcity/autoware.git ~/autoware
cd ~/autoware
source /opt/ros/jazzy/setup.bash
sudo rosdep init        # "already initialized" error is fine
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy -r
MAKEFLAGS=-j6 colcon build --symlink-install --parallel-workers 6 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=OFF
```

- The full build takes 2-4 h. Run it in the background, and lower `--parallel-workers` if the machine runs out of RAM.
- Add to `~/.bashrc` once. Order matters (see Shadowing):
  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/autoware/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml   # car; else file://$HOME/autoware/cyclonedds.xml
  ```
- `CYCLONEDDS_URI` is required. Without multicast (Wi-Fi, or `lo` without the multicast flag) CycloneDDS allows
  only 10 participants per machine. Autoware starts 30+, so later nodes die with
  `Failed to find a free participant index for domain 0` / `RCLError: error creating node`.
  [cyclonedds.xml](cyclonedds.xml) binds to loopback and allows 200 participants. It is single-machine only:
  another machine (e.g. RViz on a laptop) will not see the topics.
- On the car, the drivers' install script copies it to `/etc/ros/cyclonedds.xml`. The driver service and every shell
  must use that **same file**: when the stack and the drivers use different DDS configs, they don't see each other's
  topics. The `~/mcity_demos/run_*_demo.sh` scripts export it (and fall back to `~/autoware/cyclonedds.xml` with a warning).
- Real car only: the sensor/DBW drivers are not started by hand. See "Car drivers and sensor health" below.

Verify in a new terminal:
```bash
ros2 pkg prefix mcity_mache_bywire   # -> ~/autoware/install/mcity_mache_bywire
ros2 pkg prefix preview_control      # -> ~/autoware/install/preview_control
python3 -c "import ds_dbw_msgs.msg as m; assert hasattr(m.SteeringCmd, 'CONTROL_LANE_KEEP'); print(m.__file__)"  # -> /opt/ros/jazzy/...
```

After the first build, rebuild only what changed: `colcon build --symlink-install --packages-select <pkg>`.
For a package whose dependencies changed, use `--packages-up-to <pkg>`.

## Mcity packages (`src/mcity/`)
- `mcity_mache_bywire`: Python node that turns `/mcity/vehicle_control` into Dataspeed throttle, brake,
  steering and gear commands, and publishes `/mcity/vehicle_state`.
  `ros2 run mcity_mache_bywire mcity_mache_bywire` uses the code defaults (`max_throttle` 0.45).
  `ros2 launch mcity_mache_bywire mcity_mache_bywire.launch.py` sets `max_throttle` 1.0, `max_speed` 20, `max_lat_acc` 2.5.
  Its `CMakeLists.txt` is a stale leftover. The package is `ament_python` and builds from `setup.py`.
- `ds_dbw_msgs` comes from apt (`ros-jazzy-ds-dbw-msgs` >= 2.4.3; rosdep installs it). The workspace used to carry a
  patched 2.4.1 copy for `SteeringCmd.control_mode` (`CONTROL_LANE_KEEP`), `lane_keep_deny` and `SteerFord*`. apt 2.4.3
  has all of these, and `ds_dbw_can`'s `dbw_node` uses the apt definitions. Do not re-vendor an older copy: `GearReport`,
  `ThrottleReport` and `WheelSpeeds` changed layout (e.g. `WheelSpeeds.front_left` became `front_left_rad_s`/`front_left_m_s`),
  so a mismatched copy decodes the car's reports wrong without raising an error.
- `dbw_ford_msgs` 2.1.3: vendored for `ThrottleInfoReport`.
- `mcity_msgs`: `Control`, `VehicleState`, `VehiclePlanning`.
- `preview_control`: lateral LQR + speed PI controller. Read `src/mcity/preview_control/CLAUDE.md` before touching it.
- `stanley_control`, `pursuit_control`: alternative controllers. They publish the same topic as preview_control, so run only one.
- `mcity_route`, `autoware_cosim`, `gnss_decoder`, `mcity_abc`: route and engagement scripts, plus the CARLA/TeraSim
  co-sim bridge. `redis_client/` is loose C++ source, not a ROS package.

## Real-car command chain
`/mcity/vehicle_planning` + `/mcity/cav_pose` + `/mcity/vehicle_state` -> controller
(`ros2 launch preview_control mache_control.launch.py`) -> `/mcity/preview_control/vehicle_control`
-> relay `ros2 run mcity_vigrade mcity_preview_control` -> `/mcity/vehicle_control`
-> `mcity_mache_bywire` -> Dataspeed `ds_dbw_can` -> car.

The relay is **not in this repo**. It lives in the separate `mcity_demos` workspace (not a git repo; the
`run_*_demo.sh` scripts there start the whole stack) and needs `terasim_cosim` and a Redis server. It forwards
ViGrade's command when the Redis control-override `FLAG` is `"True"`, and Autoware's preview_control command when
`FLAG` is `"False"` **or missing**. The demo scripts run `redis-cli flushall`, so they start under Autoware control.
Ask the user where to get it if the real-car chain is needed.

## Car drivers and sensor health
- `ros2-autostart.service` (systemd, starts at boot; `/usr/local/bin/ros2-startup.sh`) sources `~/ros_ws/install`
  and launches `ds_dbw_can dbw.launch.xml`, `oxts run.py` (OXTS INS) and `rslidar_sdk start.py`, with
  `CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml`. That code and its docs belong to `~/vehicle-ros-drivers`
  (`jazzy/README.md`); the OXTS driver is the upstream `oxts_ros2_driver` clone in `~/ros_ws/src`.
  Restart: `sudo systemctl restart ros2-autostart.service`. The user has to run sudo.
- **OXTS `pub_path_rate` must stay 0** (`~/ros_ws/src/oxts_ros2_driver/oxts_ins/config/default.yaml`; the upstream
  default 100 is wrong for us). With 100, `/ins/path` republishes its whole history every 10 ms and grows forever.
  On 2026-11-20 oxts_ins reached 97 % CPU after ~2 h, and `/ins/imu` fell from 100 Hz to ~10 Hz and `/ins/nav_sat_fix`
  to ~3 Hz with 1 s gaps. `gnss_decoder` publishes `/mcity/cav_pose` once per fix, so the pose froze for up to 1 s
  and then jumped 4-6 m. preview_control then weaved at its steering rate limit and left the path (1.97 m).
  A service restart only clears it temporarily.
- Before tuning or blaming the controller, check that the inputs are healthy (with `CYCLONEDDS_URI` set):
  `/ins/imu` and `/ins/odometry` ~100 Hz, `/ins/nav_sat_fix` ~50 Hz, `/mcity/cav_pose` and `/mcity/vehicle_state`
  ~50 Hz, max gap < 50 ms, `oxts_ins` ~5 % CPU in `top`. Drives recorded before 2026-11-20 12:47 had a degraded pose
  feed, so don't use them to judge controller tuning.
- `cav_pose` has no dead-reckoning between fixes: it is only as smooth as `/ins/nav_sat_fix`.
- No GNSS fix (`/ins/nav_sat_fix` missing) leaves Autoware waiting for localization initialization. Check the INS first.

## Mach-E facts
- Steering engages (`report.enabled`) only when `SteeringCmd.control_mode = CONTROL_LANE_KEEP`.
- `VehicleState.yaw_rate` is always 0 on the Mach-E. Derive yaw rate from steering angle and speed instead.
- The brake command is 0..1, which by-wire sends as 0..100 %. Any brake > 0 cuts throttle.
- Zero throttle is regen braking, about -0.3 m/s^2 (not coasting). preview_control's `lift_decel` handles this.
- `VehicleState.speed_x` = wheel rad/s x 0.3365 m (MKZ wheel radius) reads ~5.9 % low against the INS. The Mach-E
  wheel diameter is ~0.713 m. Not changed yet: the controller tuning and `max_speed` were done with the low reading.
- Measured steering (2026-11-20): delay 0.03 s, actuator lag 0.094 s, tracks the command 1:1, steering ratio 14.43,
  understeer 1.4 deg/g.

## Route scripts and the mission planner (`~/mcity_demos/src/mcity_vigrade`)
- Route scripts send `SetRoutePoints` (goal + waypoints, map frame) and then switch to autonomous mode.
  `mcity_full_route1` (2026-11-20, from the manual baseline drive around the facility) chains loop-free segments
  A -> B: on `ARRIVED_GOAL` it waits `hand_over_wait` (3 s), calls `/planning/mission_planning/clear_route`, and
  sets the next segment. `laps` 0 = endless. Only the route script does this; no Autoware module was changed.
- The mission planner rejects a route when:
  - the goal footprint (sample_vehicle: front 3.79 m and rear 1.1 m from the rear axle, half width 0.948 m)
    leaves the goal lanelet: "Goal's footprint exceeds lane". Put goals on straight lanelet centrelines.
  - any lanelet repeats in the route, counting same-direction lateral neighbours: "Loop detected within route!".
    So a closed lap cannot be a single route.
  - a route is already set: `SetRoutePoints` works only in state UNSET, so clear the route after `ARRIVED_GOAL`.
- Map frame: map xy = UTM(lat, lon) + (-277497.10, -4686518.71); origin LocalCartesianUTM (42.299138, -83.699245).
  The Autoware pose is the vehicle centre shifted 1.5 m back (rear axle).
- Known issue: the route scripts never subscribe to `/mcity/vehicle_state`, so their by-wire checks see no data.

## Recording drives
- Record every test in its own bag under `~/bags` with `CYCLONEDDS_URI` set:
  `ros2 bag record -o ~/bags/<name>_$(date +%Y%m%d_%H%M%S) --include-hidden-topics --regex
  "^/(mcity/.*|vehicle/.*|ins/(odometry|nav_sat_fix|imu)|autoware/state|rosout|planning/scenario_planning/trajectory|planning/mission_planning/route|system/operation_mode/state)$"`.
  Stop it with SIGINT or SIGTERM. A recorder started with `nohup ... &` ignores SIGINT.
- Calibration drives: the skill's `record_drive.sh`, which saves to `~/mache_calib/drives/bags`.
- References from 2026-11-20 in `~/bags`: `baseline_manual_20261120_112617` (in `~/mache_calib/drives/bags`) is the
  manual drive around the facility. `highway_clean_20261120_125120` is the first clean-INS autonomous highway run:
  straights 0.14 m mean |ey|, curves cut ~0.4 m. `steer_lowspeed_20261120_131707` is the run with all of that day's
  preview_control fixes (curves 0.24 m mean |ey|). Open issues are listed in `src/mcity/preview_control/CLAUDE.md`.

## Shadowing (common source of "my change did nothing")
Several workspaces install packages or libraries with the same names. Whichever is sourced **last** wins.
- Old standalone workspaces `~/mcity_mache_bywire` and `~/vehicle_ws` contain copies of `mcity_mache_bywire`,
  `ds_dbw_msgs`, `dbw_ford_msgs` and `mcity_msgs`. This repo replaces them, so don't source them on a new machine.
- `~/mcity_demos` installs another `libpreview_control_component.so`. `mache_control.launch.py` works
  around it, but a plain `ros2 run preview_control` loads the wrong library.
- Check with `ros2 pkg prefix <pkg>` and `python3 -c "import <pkg>; print(<pkg>.__file__)"`.

## Skills
- `.claude/skills/preview-gain-calibration/`: preview_control gain tables for the Mach-E (sys-id drives, fitting,
  regeneration, validation).
