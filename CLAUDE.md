# Mcity Autoware (Ford Mach-E)

Mcity's fork of Autoware Universe (Oct 2023 base), ROS 2 Humble on Ubuntu 22.04. Mcity code lives in
`src/mcity/`. The real car is a Ford Mach-E driven through a Dataspeed DBW kit.

## Fresh-machine setup

Clone from **`https://github.com/mcity/autoware.git`** (remote `origin`). The README's
`michigan-traffic-lab/autoware` URL (remote `mtl`) is far behind and has none of the Mach-E work.

Prerequisites: Ubuntu 22.04 with ROS 2 Humble Desktop installed in `/opt/ros/humble`. The user has to run the `sudo` steps.

```bash
sudo apt-get install -y libhiredis-dev libgeographic-dev ccache python3-rosdep2 \
  python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp
git clone https://github.com/mcity/autoware.git ~/autoware
cd ~/autoware
source /opt/ros/humble/setup.bash
sudo rosdep init        # "already initialized" error is fine
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8
```

- The full build takes 2-4 h. Run it in the background, and lower `--parallel-workers` if the machine runs out of RAM.
- Add to `~/.bashrc` once. Order matters (see Shadowing):
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/autoware/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
- Real car only: install the Dataspeed CAN driver with `sudo apt-get install -y ros-humble-ds-dbw`. Start it with
  `ros2 launch ds_dbw_can dbw.launch.xml`.

Verify in a new terminal:
```bash
ros2 pkg prefix mcity_mache_bywire   # -> ~/autoware/install/mcity_mache_bywire
ros2 pkg prefix preview_control      # -> ~/autoware/install/preview_control
python3 -c "import ds_dbw_msgs.msg as m; assert hasattr(m.SteeringCmd, 'CONTROL_LANE_KEEP'); print('ok')"
```

After the first build, rebuild only what changed: `colcon build --symlink-install --packages-select <pkg>`.
For a package whose dependencies changed, use `--packages-up-to <pkg>`.

## Mcity packages (`src/mcity/`)
- `mcity_mache_bywire`: Python node that turns `/mcity/vehicle_control` into Dataspeed throttle, brake,
  steering and gear commands, and publishes `/mcity/vehicle_state`.
  `ros2 run mcity_mache_bywire mcity_mache_bywire` uses the code defaults (`max_throttle` 0.45).
  `ros2 launch mcity_mache_bywire mcity_mache_bywire.launch.py` sets `max_throttle` 1.0, `max_speed` 20, `max_lat_acc` 2.5.
  Its `CMakeLists.txt` is a stale leftover. The package is `ament_python` and builds from `setup.py`.
- `ds_dbw_msgs`: **locally patched** Dataspeed 2.4.1. It adds `SteeringCmd.control_mode`
  (`CONTROL_LANE_KEEP`), `lane_keep_deny` diagnostics and the `SteerFord*` messages. The apt
  `ros-humble-ds-dbw-msgs` lacks these, so the workspace copy must stay on top of `/opt/ros`. Do not replace it with upstream.
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

The relay is **not in this repo**. It lives in the separate `mcity_demos` workspace, needs `terasim_cosim` and a
Redis server, and forwards commands only while the Redis control-override `FLAG` is `"False"`.
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
