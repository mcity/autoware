"""
Pure Pursuit lateral controller.

Formula:
    delta = atan(2 * L * sin(alpha) / Ld)

where:
    L     = wheelbase [m]
    alpha = angle from vehicle heading to the lookahead point direction [rad]
    Ld    = lookahead distance [m]  (speed-adaptive: k * v + Ld_min)

Sign convention:
    lookahead point to the LEFT  of vehicle heading → alpha > 0 → steer LEFT  (positive delta)
    lookahead point to the RIGHT of vehicle heading → alpha < 0 → steer RIGHT (negative delta)
"""

import math
from typing import List, Tuple

from .vehicle_params import STEERING_RATIO, MAX_WHEEL_ANGLE, WHEELBASE


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PurePursuitController:
    """
    Pure Pursuit steering controller.

    Parameters
    ----------
    k        : Speed-adaptive lookahead gain [s].  Ld = k * v + Ld_min.
    Ld_min   : Minimum lookahead distance [m].  Used at low / zero speed.
    Ld_max   : Maximum lookahead distance [m].  Caps Ld at high speed.
    wheelbase: Rear-to-front axle distance [m].
    """

    def __init__(
        self,
        k: float = 0.8,
        Ld_min: float = 2.0,
        Ld_max: float = 8.0,
        wheelbase: float = WHEELBASE,
        k_ey: float = 0.0,
        k_ephi: float = 0.0,
    ):
        self.k         = k
        self.Ld_min    = Ld_min
        self.Ld_max    = Ld_max
        self.wheelbase = wheelbase
        self.k_ey      = k_ey
        self.k_ephi    = k_ephi

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
        ey: float = 0.0,
        ephi: float = 0.0,
    ) -> float:
        """
        Returns the steering wheel angle command [rad].

        Steps
        -----
        1. Compute speed-adaptive lookahead distance.
        2. Find the lookahead point on the path at distance Ld from vehicle.
        3. Compute alpha (angle from vehicle heading to lookahead direction).
        4. Apply Pure Pursuit formula → wheel angle → steering wheel angle.
        """
        if len(x_vec) < 2:
            return 0.0

        # ── Step 1: lookahead distance ───────────────────────────────────────
        Ld = max(self.Ld_min, min(self.Ld_max, self.k * speed_x + self.Ld_min))

        # ── Step 2: lookahead point on path ──────────────────────────────────
        lx, ly, _ = self._find_lookahead_point(pos_x, pos_y, x_vec, y_vec, Ld, start_idx)

        # ── Step 3: alpha — angle from heading to lookahead direction ─────────
        dx = lx - pos_x
        dy = ly - pos_y
        lookahead_angle = math.atan2(dy, dx)
        alpha = normalize_angle(lookahead_angle - (yaw + heading_offset))

        # ── Step 4: Pure Pursuit law ──────────────────────────────────────────
        wheel_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)

        # Cross-track error correction — Stanley-style ey/v term.
        # ey < 0: vehicle left of path → steers right (negative correction).
        if self.k_ey > 1e-9:
            v_eff = max(speed_x, 0.5)
            wheel_angle += math.atan2(self.k_ey * ey, v_eff)

        # Heading error correction — proportional to ephi so the car quickly
        # unwinds residual heading error after a lane change instead of relying
        # solely on the lookahead geometry.
        # ephi > 0: path more CCW than car heading → steer left (positive).
        # ephi < 0: car still angled from lane change → steers back to straight.
        if abs(self.k_ephi) > 1e-9:
            wheel_angle += self.k_ephi * ephi

        wheel_angle = max(-MAX_WHEEL_ANGLE, min(MAX_WHEEL_ANGLE, wheel_angle))

        steering_wheel_angle = wheel_angle * STEERING_RATIO
        return steering_wheel_angle

    # ── helper ───────────────────────────────────────────────────────────────

    def _find_lookahead_point(
        self,
        pos_x: float, pos_y: float,
        x_vec: List[float], y_vec: List[float],
        Ld: float,
        start_idx: int,
    ) -> Tuple[float, float, int]:
        """
        Walk forward along the path from start_idx by arc-length until the
        accumulated path distance reaches Ld, then interpolate.  Falls back
        to the last path point if the remaining path is shorter than Ld.

        Arc-length (not Euclidean vehicle→point distance) is used so that
        lateral offset (e.g. during a lane change) does not cause Ld to fire
        prematurely, which would make the effective lookahead collapse to the
        nearest path point and prevent steering from returning to centre.
        """
        search_from = max(0, start_idx)
        arc = 0.0

        for i in range(search_from, len(x_vec) - 1):
            seg_dx = x_vec[i + 1] - x_vec[i]
            seg_dy = y_vec[i + 1] - y_vec[i]
            seg_len = math.hypot(seg_dx, seg_dy)

            if arc + seg_len >= Ld:
                frac = (Ld - arc) / seg_len if seg_len > 1e-9 else 0.0
                lx = x_vec[i] + frac * seg_dx
                ly = y_vec[i] + frac * seg_dy
                return lx, ly, i + 1

            arc += seg_len

        # Path shorter than Ld — track the last available point
        return x_vec[-1], y_vec[-1], len(x_vec) - 1
