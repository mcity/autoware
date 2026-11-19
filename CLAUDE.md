# Mcity Autoware (Ford Mach-E)

Mcity's fork of Autoware Universe (Oct 2023 base). Mcity code lives in `src/mcity/`. The real car is a
Ford Mach-E driven through a Dataspeed DBW kit.

**Branch `autoware_ubuntu_24` targets ROS 2 Jazzy on Ubuntu 24.04** (gcc 13, Python 3.12). `main` stays on
Humble / Ubuntu 22.04. Jazzy port notes:
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
git clone -b autoware_ubuntu_24 https://github.com/mcity/autoware.git ~/autoware
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
  export CYCLONEDDS_URI=file://$HOME/autoware/cyclonedds.xml
  ```
- `CYCLONEDDS_URI` is required. Without multicast (Wi-Fi, or `lo` without the multicast flag) CycloneDDS allows
  only 10 participants per machine. Autoware starts 30+, so later nodes die with
  `Failed to find a free participant index for domain 0` / `RCLError: error creating node`.
  [cyclonedds.xml](cyclonedds.xml) binds to loopback and allows 200 participants. It is single-machine only:
  another machine (e.g. RViz on a laptop) will not see the topics.
- Real car only: install the Dataspeed CAN driver with `sudo apt-get install -y ros-jazzy-ds-dbw`. Start it with
  `ros2 launch ds_dbw_can dbw.launch.xml`.

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

## Mach-E facts
- Steering engages (`report.enabled`) only when `SteeringCmd.control_mode = CONTROL_LANE_KEEP`.
- `VehicleState.yaw_rate` is always 0 on the Mach-E. Derive yaw rate from steering angle and speed instead.
- The brake command is 0..1, which by-wire sends as 0..100 %. Any brake > 0 cuts throttle.

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
