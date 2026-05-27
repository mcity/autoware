"""
Path processing: downsampling, closest-point search, lateral/heading error.
Mirrors path_process.cpp faithfully.
"""

import math
from typing import List, Tuple, Optional


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathProcessor:
    """
    Holds the processed trajectory and computes path errors each tick.

    Call process_path() once when a new VehiclePlanning message arrives,
    then call run() every control tick to get updated errors.
    """

    def __init__(
        self,
        max_ey: float,
        max_ephi: float,
        heading_offset: float,
        heading_lookahead_points: int,
        lateral_offset: float,
    ):
        self.max_ey = max_ey
        self.max_ephi = max_ephi
        self.heading_offset = heading_offset
        self.heading_lookahead_points = heading_lookahead_points
        self.lateral_offset = lateral_offset

        # Processed trajectory (updated on each new planning message)
        self.x_vec:   List[float] = []
        self.y_vec:   List[float] = []
        self.vd_vec:  List[float] = []
        self.ori_vec: List[float] = []

        # Latest computed results
        self.closest_index: int   = 0
        self.ey:    float = 0.0   # lateral error  [m]
        self.ephi:  float = 0.0   # heading error  [rad]
        self.vd:    float = 0.0   # desired speed  [m/s]
        self.slope: float = 0.0

    # ── called from pathCB ───────────────────────────────────────────────────

    def process_path(
        self,
        x_vec: List[float],
        y_vec: List[float],
        vd_vec: List[float],
        ori_vec: List[float],
        time_resolution: float,
        preview_time: float,
        desired_time_resolution: float,
    ):
        self.x_vec   = list(x_vec)
        self.y_vec   = list(y_vec)
        self.vd_vec  = list(vd_vec)
        self.ori_vec = list(ori_vec)
        self._downsampling(preview_time, desired_time_resolution)

    # ── called from on_timer ─────────────────────────────────────────────────

    def run(
        self,
        pos_x: float, pos_y: float,
        qx: float, qy: float, qz: float, qw: float,
        speed_x: float,
    ):
        if len(self.x_vec) < 3:
            return

        idx = self._get_closest_index(pos_x, pos_y)
        self.closest_index = idx

        self.vd    = self._get_desired_velocity(idx, speed_x)
        self.ephi  = self._get_orientation_error(idx, qx, qy, qz, qw)
        self.ey    = self._get_lateral_error(idx, pos_x, pos_y)
        self.slope = 0.0

    def remaining_size(self) -> int:
        return len(self.x_vec)

    # ── private helpers ──────────────────────────────────────────────────────

    def _downsampling(self, preview_time: float, desired_dt: float):
        ds_x, ds_y, ds_vd, ds_ori = [], [], [], []
        accumulated = 0.0

        for i in range(len(self.x_vec) - 1):
            dx = self.x_vec[i + 1] - self.x_vec[i]
            dy = self.y_vec[i + 1] - self.y_vec[i]
            dist = math.sqrt(dx * dx + dy * dy)
            vd = max(1.5, float(self.vd_vec[i]))
            accumulated += dist / vd

            if accumulated >= desired_dt:
                ds_x.append(self.x_vec[i])
                ds_y.append(self.y_vec[i])
                ds_vd.append(self.vd_vec[i])
                ds_ori.append(self.ori_vec[i])
                accumulated = 0.0

            if len(ds_x) >= int(preview_time / desired_dt):
                break

        self.x_vec   = ds_x
        self.y_vec   = ds_y
        self.vd_vec  = ds_vd
        self.ori_vec = ds_ori

    def _get_closest_index(self, pos_x: float, pos_y: float) -> int:
        min_dist = float('inf')
        best_idx = 1
        for i in range(1, len(self.x_vec) - 1):
            d = math.sqrt(
                (self.x_vec[i] - pos_x) ** 2
                + (self.y_vec[i] - pos_y) ** 2
            )
            if d < min_dist:
                min_dist = d
                best_idx = i
        return best_idx

    def _get_desired_velocity(self, idx: int, current_v: float) -> float:
        vd = float(self.vd_vec[idx])

        if current_v <= 0.05 and self.vd_vec[-1] <= 0.5:
            return 0.0

        if current_v >= vd and vd <= 2.0:
            min_diff = float('inf')
            for i in range(idx, len(self.vd_vec)):
                diff = abs(current_v + 0.1 - self.vd_vec[i])
                if diff < min_diff:
                    min_diff = diff
                    vd = float(self.vd_vec[i])

        return vd

    def _get_orientation_error(
        self, idx: int,
        qx: float, qy: float, qz: float, qw: float
    ) -> float:
        yaw = quaternion_to_yaw(qx, qy, qz, qw)

        look = idx + self.heading_lookahead_points
        traj_heading = float(
            self.ori_vec[look] if look < len(self.ori_vec) else self.ori_vec[idx]
        )

        err = traj_heading - (yaw + self.heading_offset)
        return normalize_angle(err)

    def _get_lateral_error(self, idx: int, pos_x: float, pos_y: float) -> float:
        x_pre  = self.x_vec[idx - 1]
        y_pre  = self.y_vec[idx - 1]
        x_next = self.x_vec[idx + 1]
        y_next = self.y_vec[idx + 1]

        dx = x_next - x_pre
        dy = y_next - y_pre
        denom = math.sqrt(dy * dy + dx * dx)
        if denom < 1e-9:
            return 0.0

        lateral_error = abs(dy * pos_x - dx * pos_y + x_next * y_pre - y_next * x_pre) / denom

        cross = dx * (pos_y - y_pre) - dy * (pos_x - x_pre)
        if cross > 0:
            lateral_error = -lateral_error

        return lateral_error + self.lateral_offset
