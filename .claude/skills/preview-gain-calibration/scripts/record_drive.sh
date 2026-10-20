#!/usr/bin/env bash
# Record a calibration bag while driving the MachE around (manual or by-wire).
#   record_drive.sh [name]          -> ~/mache_calib/drives/bags/<name or drive_YYYY-mm-dd_HHMMSS>
# Ctrl-C to stop, then: python3 update_from_drive.py ~/mache_calib/drives/bags/<name>
set -euo pipefail
STORE=${MACHE_CALIB_STORE:-$HOME/mache_calib/drives}
NAME=${1:-drive_$(date +%F_%H%M%S)}
mkdir -p "$STORE/bags"
source "$HOME/autoware/install/setup.bash"

for t in /mcity/vehicle_state /ins/imu; do
    if ! timeout 3 ros2 topic echo --once "$t" >/dev/null 2>&1; then
        echo "WARNING: no messages on $t - the fits need it" >&2
    fi
done

exec ros2 bag record -o "$STORE/bags/$NAME" \
    /mcity/vehicle_state /mcity/vehicle_control /mcity/preview_control/vehicle_control \
    /mcity/cav_pose /mcity/vehicle_planning \
    /ins/imu /ins/odometry \
    /vehicle/steering/report /vehicle/steering/cmd /vehicle/wheel_speeds
