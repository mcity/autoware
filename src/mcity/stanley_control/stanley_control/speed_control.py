"""
Cascaded PID speed controller for Mustang Mach-E.

Architecture
------------
Outer loop  (velocity controller):
    error    = vd - speed_x
    output   = acc_desired  [m/s²]

Inner loop  (acceleration controller):
    error    = acc_desired - acc_measured
    output   = pedal_raw   (positive → throttle, negative → brake)

acc_measured is estimated by finite-differencing speed_x and then
low-pass filtering to suppress 50 Hz sensor noise.

No vehicle-specific lookup tables are required — only six PID gains
and one filter coefficient, all tunable at runtime via ROS parameters.

Retained from the original speed_control.cpp
--------------------------------------------
  • Slope feedforward  (gravity compensation via 9.8·sin(slope))
  • Planner acc feedforward  (acc_d)
  • Gear management  (force GEAR_DRIVE when vd > 0)
  • Integrator wind-up guards
  • estop level overrides (LOW / MEDIUM / HIGH)
  • Smooth brake ramp filter  (β = 0.95)
  • set_stop() for controlled halt
"""

import math

from .vehicle_params import (
    MAX_THROTTLE, MAX_BRAKE, MAX_SPEED, MAX_ACC, MAX_DCC,
)

# ── Gear constants ────────────────────────────────────────────────────────────
GEAR_NONE    = 0
GEAR_PARK    = 1
GEAR_REVERSE = 2
GEAR_NEUTRAL = 3
GEAR_DRIVE   = 4
GEAR_LOW     = 5

# ── Estop levels ──────────────────────────────────────────────────────────────
ESTOP_NONE   = 0
ESTOP_LOW    = 1
ESTOP_MEDIUM = 2
ESTOP_HIGH   = 3

# ── Brake pedal applied when stopping ─────────────────────────────────────────
_BRAKE_HOLD = 0.22   # enough to hold Mach-E stationary


class CascadedPIDSpeedController:
    """
    Two-loop speed controller.

    Parameters (all tunable, defaults are Mach-E starting points)
    -------------------------------------------------------------
    kp_v, ki_v, kd_v   Outer PID gains  (speed  → desired acceleration)
    kp_a, ki_a, kd_a   Inner PID gains  (acc error → pedal fraction)
    acc_filter_alpha    Low-pass α for the acceleration estimate [0–1).
                        Higher α = more smoothing, more lag.
    frequency           Control loop rate [Hz].
    """

    def __init__(
        self,
        kp_v: float = 1.5,
        ki_v: float = 0.5,
        kd_v: float = 0.1,
        kp_a: float = 0.10,
        ki_a: float = 0.02,
        kd_a: float = 0.0,
        acc_filter_alpha: float = 0.7,
        frequency: int = 50,
    ):
        self._kp_v = kp_v
        self._ki_v = ki_v
        self._kd_v = kd_v
        self._kp_a = kp_a
        self._ki_a = ki_a
        self._kd_a = kd_a
        self._alpha  = acc_filter_alpha
        self._dt     = 1.0 / frequency
        self._freq   = frequency

        # Outer loop state
        self._v_intg     = 0.0
        self._v_err_prev = 0.0

        # Inner loop state
        self._a_intg     = 0.0
        self._a_err_prev = 0.0

        # Acceleration estimator state
        self._speed_prev = 0.0
        self._acc_filt   = 0.0

        # Auxiliary state
        self.acc_slope   = 0.0
        self._brake_prev = 0.0
        self._stop_count = 0
        self.gear_cmd    = GEAR_DRIVE

    # ── Public API ────────────────────────────────────────────────────────────

    def run(
        self,
        vd: float,
        speed_x: float,
        slope: float,
        gear_pos: int,
        estop: int,
        acc_d: float = 0.0,
    ):
        """
        Run one control tick.  Returns (throttle, brake, gear_cmd).

        Parameters
        ----------
        vd        : desired speed [m/s] from planner
        speed_x   : measured vehicle speed [m/s]
        slope     : road slope [rad]  (positive = uphill)
        gear_pos  : current gear reported by vehicle CAN
        estop     : emergency stop level (ESTOP_NONE … ESTOP_HIGH)
        acc_d     : acceleration feedforward from planner [m/s²]
        """
        speed_x = max(0.0, speed_x)
        self.acc_slope = 9.8 * math.sin(slope)

        # ── Step 1: estimate measured acceleration ────────────────────────────
        acc_raw          = (speed_x - self._speed_prev) / self._dt
        self._acc_filt   = self._alpha * self._acc_filt + (1.0 - self._alpha) * acc_raw
        self._speed_prev = speed_x
        acc_measured     = self._acc_filt

        # ── Step 2: outer loop — velocity → desired acceleration ──────────────
        v_err      = vd - speed_x
        acc_desire = self._outer_pid(v_err, acc_d)

        # ── Step 3: inner loop — acc error → pedal ────────────────────────────
        a_err     = acc_desire - acc_measured
        pedal_raw = self._inner_pid(a_err)

        # ── Step 4: split pedal into throttle / brake ─────────────────────────
        if pedal_raw >= 0.0:
            throttle = min(pedal_raw, MAX_THROTTLE)
            brake    = 0.0
        else:
            throttle = 0.0
            brake    = min(-pedal_raw, MAX_BRAKE)

        # ── Step 5: integrator wind-up guards ────────────────────────────────
        if gear_pos != GEAR_DRIVE:
            self._v_intg = 0.0
            self._a_intg = 0.0
        if vd == 0.0 and self._v_intg < 0.0:
            self._v_intg = 0.0
        if speed_x == 0.0 and self._v_intg > 0.0:
            self._v_intg = 0.0

        # ── Step 6: gear management ───────────────────────────────────────────
        if gear_pos != GEAR_DRIVE and vd > 0.1:
            self._v_intg = 0.0
            self._a_intg = 0.0
            throttle     = 0.0
            brake        = 0.20
            self.gear_cmd = GEAR_DRIVE
        else:
            self.gear_cmd = GEAR_DRIVE

        # ── Step 7: speed cap ─────────────────────────────────────────────────
        if speed_x > MAX_SPEED + 0.2:
            throttle = 0.0

        # ── Step 8: low-speed launch assist ───────────────────────────────────
        if speed_x < 2.5 and vd > speed_x + 3.0 and estop == ESTOP_NONE:
            throttle = 0.30
            brake    = 0.0

        # ── Step 9: near-stop brake hold ──────────────────────────────────────
        if speed_x < 0.3 and vd < 2.0 and brake > 0.1:
            brake = _BRAKE_HOLD if vd == 0.0 else max(_BRAKE_HOLD, brake)

        # ── Step 10: pedal mutual exclusion + clamp ───────────────────────────
        throttle = max(0.0, min(MAX_THROTTLE, throttle))
        brake    = max(0.0, min(MAX_BRAKE,    brake))
        if brake    > 0.0:  throttle = 0.0
        if throttle > 0.01: brake    = 0.0

        # ── Step 11: estop overrides ──────────────────────────────────────────
        if estop != ESTOP_NONE:
            throttle, brake = self._apply_estop(estop, speed_x)

        # ── Step 12: smooth brake ramp (β = 0.95) ────────────────────────────
        beta = 0.95
        if brake >= 0.18:
            if brake > self._brake_prev:
                brake = brake * (1.0 - beta) + self._brake_prev * beta
            self._brake_prev = max(0.18, brake)

        return throttle, brake, self.gear_cmd

    def set_stop(self, speed_x: float):
        """
        Controlled halt — ignores the PID loops.
        Returns (throttle, brake, gear).
        """
        if speed_x >= 0.2:
            brake = _BRAKE_HOLD
        else:
            brake = _BRAKE_HOLD + abs(self.acc_slope) * 0.1

        gear = GEAR_DRIVE
        if speed_x == 0.0:
            self._stop_count += 1
            if self._stop_count > self._freq * 1.5:   # 1.5 s
                brake = 0.235
                gear  = GEAR_PARK
        else:
            self._stop_count = 0

        return 0.0, brake, gear

    def reset(self):
        self._v_intg     = 0.0
        self._v_err_prev = 0.0
        self._a_intg     = 0.0
        self._a_err_prev = 0.0
        self._acc_filt   = 0.0
        self._brake_prev = 0.0

    # ── Private ───────────────────────────────────────────────────────────────

    def _outer_pid(self, v_err: float, acc_d: float) -> float:
        """Velocity PID → desired acceleration [m/s²]."""
        # Proportional
        p = self._kp_v * v_err

        # Integral (clamped anti-windup)
        self._v_intg += v_err * self._dt
        self._v_intg  = max(-2.0, min(2.0, self._v_intg))
        i = self._ki_v * self._v_intg

        # Derivative on measurement (avoids setpoint-change kick)
        d = -self._kd_v * (v_err - self._v_err_prev) / self._dt
        self._v_err_prev = v_err

        acc_desire = p + i + d + acc_d + self.acc_slope
        return max(MAX_DCC, min(MAX_ACC, acc_desire))

    def _inner_pid(self, a_err: float) -> float:
        """Acceleration PID → raw pedal fraction (signed)."""
        p = self._kp_a * a_err

        self._a_intg += a_err * self._dt
        self._a_intg  = max(-MAX_BRAKE, min(MAX_THROTTLE, self._a_intg))
        i = self._ki_a * self._a_intg

        d = self._kd_a * (a_err - self._a_err_prev) / self._dt
        self._a_err_prev = a_err

        return p + i + d

    def _apply_estop(self, estop: int, speed_x: float):
        if estop == ESTOP_HIGH:
            brake = 0.30 if speed_x >= 0.5 else 0.235
            return 0.0, max(brake, 0.25)
        elif estop == ESTOP_MEDIUM:
            return 0.0, 0.245
        else:  # ESTOP_LOW
            return 0.0, _BRAKE_HOLD


# Alias kept so the node import doesn't need changing
SpeedController = CascadedPIDSpeedController
