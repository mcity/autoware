"""
Stanley lateral controller.

Formula:
    delta = heading_error + atan(k * cte_front / max(v, v_min))

where cte_front is the signed cross-track error measured at the **front axle**
(not the CoM), which is the key distinction of the Stanley method.

Sign convention (matches path_process.py / C++ original):
    cte_front > 0  → vehicle is to the RIGHT of path → steer LEFT  (positive delta)
    cte_front < 0  → vehicle is to the LEFT  of path → steer RIGHT (negative delta)
"""

import math
from typing import List

from .vehicle_params import STEERING_RATIO, MAX_WHEEL_ANGLE, WHEELBASE


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class StanleyController:
    """
    Stateless Stanley steering controller.

    Parameters
    ----------
    k       : Stanley gain on the cross-track error term.
    k_soft  : Softening speed [m/s] — avoids division by zero at standstill.
    wheelbase : Distance from rear to front axle [m].
    """

    def __init__(
        self,
        k: float = 0.5,
        k_soft: float = 1.0,
        wheelbase: float = WHEELBASE,
    ):
        self.k         = k
        self.k_soft    = k_soft
        self.wheelbase = wheelbase

    def compute_steering(
        self,
        pos_x: float,
        pos_y: float,
        yaw: float,
        speed_x: float,
        x_vec: List[float],
        y_vec: List[float],
        ori_vec: List[float],
        heading_offset: float = 0.0,
        start_idx: int = 0,
    ) -> float:
        """
        Returns the steering wheel angle command [rad].

        Steps
        -----
        1. Project vehicle position to front axle.
        2. Find the closest path point to the front axle.
        3. Compute signed cross-track error at front axle.
        4. Compute heading error.
        5. Apply Stanley formula → wheel angle → steering wheel angle.
        """
        if len(x_vec) < 2:
            return 0.0

        # ── Step 1: front axle position ──────────────────────────────────────
        fx = pos_x + self.wheelbase * math.cos(yaw)
        fy = pos_y + self.wheelbase * math.sin(yaw)

        # ── Step 2: closest path point to front axle ─────────────────────────
        # Search forward from start_idx so the controller never snaps back onto
        # path points the vehicle has already passed (e.g. after exiting a turn).
        search_from = max(0, start_idx - 1)
        min_dist = float('inf')
        target_idx = search_from
        for i in range(search_from, len(x_vec)):
            d = math.sqrt((x_vec[i] - fx) ** 2 + (y_vec[i] - fy) ** 2)
            if d < min_dist:
                min_dist = d
                target_idx = i

        # ── Step 3: signed cross-track error at front axle ───────────────────
        cte = self._signed_cte(fx, fy, x_vec, y_vec, target_idx)

        # ── Step 4: heading error ─────────────────────────────────────────────
        path_heading = float(ori_vec[target_idx])
        heading_err  = normalize_angle(path_heading - (yaw + heading_offset))

        # ── Step 5: Stanley law ───────────────────────────────────────────────
        effective_speed = max(speed_x, self.k_soft)
        cte_term        = math.atan2(self.k * cte, effective_speed)

        wheel_angle = normalize_angle(heading_err + cte_term)
        wheel_angle = max(-MAX_WHEEL_ANGLE, min(MAX_WHEEL_ANGLE, wheel_angle))

        steering_wheel_angle = wheel_angle * STEERING_RATIO
        return steering_wheel_angle

    # ── helper ───────────────────────────────────────────────────────────────

    def _signed_cte(
        self,
        fx: float, fy: float,
        x_vec: List[float], y_vec: List[float],
        idx: int,
    ) -> float:
        """
        Signed perpendicular distance from (fx,fy) to the path segment
        surrounding idx.  Positive = vehicle right of path.
        Mirrors get_lateral_error() in path_process.cpp.
        """
        n = len(x_vec)

        # Use neighbouring points to define the local tangent
        i_pre  = max(0, idx - 1)
        i_next = min(n - 1, idx + 1)

        if i_pre == i_next:
            return 0.0

        x_pre  = x_vec[i_pre]
        y_pre  = y_vec[i_pre]
        x_next = x_vec[i_next]
        y_next = y_vec[i_next]

        dx    = x_next - x_pre
        dy    = y_next - y_pre
        denom = math.sqrt(dx * dx + dy * dy)
        if denom < 1e-9:
            return 0.0

        dist = abs(dy * fx - dx * fy + x_next * y_pre - y_next * x_pre) / denom

        cross = dx * (fy - y_pre) - dy * (fx - x_pre)
        if cross > 0:
            dist = -dist   # vehicle is left of path → negative cte

        return dist
