#!/usr/bin/env python3
"""Repeatable by-wire steering excitation for system identification (MachE / any mcity_bywire car).

Publishes mcity_msgs/Control on /mcity/vehicle_control at 50 Hz, exactly like preview_control:
  * holds a target speed with a gentle PI throttle loop,
  * once speed has settled, plays a steering pattern (steering-wheel degrees),
  * then centres the wheel and brakes to a stop.

SAFETY: closed course, safety driver hands near the wheel, nothing else publishing on
/mcity/vehicle_control (stop preview_control / pursuit / stanley first). The node stops
commanding immediately if by-wire drops or any steer/brake/throttle override is reported.

    source ~/autoware/install/setup.bash
    python3 steer_excitation.py --pattern step  --amps 30 60 90 --speed 4
    python3 steer_excitation.py --pattern chirp --amps 30 --f0 0.2 --f1 2.0 --duration 30 --speed 4
    python3 steer_excitation.py --pattern sine  --amps 20 --freqs 0.3 0.6 1.0 1.5 --cycles 5 --speed 10
"""
import argparse
import math

import rclpy
from rclpy.node import Node

from mcity_msgs.msg import Control, VehicleState

GEAR_DRIVE = 4
DT = 0.02


def build_pattern(a):
    """Returns list of steering-wheel angles [deg], one per DT."""
    seq = []
    if a.pattern == "step":
        n = int(round(a.hold / DT))
        for _ in range(a.repeat):
            for amp in a.amps:
                for target in (amp, 0.0, -amp, 0.0):
                    seq += [target] * n
    elif a.pattern == "chirp":
        T, amp = a.duration, a.amps[0]
        for k in range(int(T / DT)):
            t = k * DT
            phase = 2 * math.pi * (a.f0 * t + (a.f1 - a.f0) * t * t / (2 * T))
            seq.append(amp * math.sin(phase))
        seq += [0.0] * int(2.0 / DT)
    elif a.pattern == "sine":
        amp = a.amps[0]
        for f in a.freqs:
            n = int(round(a.cycles / f / DT))
            seq += [amp * math.sin(2 * math.pi * f * k * DT) for k in range(n)]
            seq += [0.0] * int(2.0 / DT)
    return seq


class SteerExcitation(Node):
    def __init__(self, a):
        super().__init__("steer_excitation")
        self.a = a
        self.pattern = build_pattern(a)
        self.pub = self.create_publisher(Control, "/mcity/vehicle_control", 10)
        self.create_subscription(VehicleState, "/mcity/vehicle_state", self.vs_cb, 10)
        self.vs = None
        self.phase = "wait_bywire"   # -> settle -> play -> stop -> done
        self.k = 0
        self.settled_t = 0.0
        self.settle_elapsed = 0.0
        self.integ = 0.0
        self.count = 0
        self.create_timer(DT, self.on_timer)
        self.get_logger().info(
            f"pattern={a.pattern} steps={len(self.pattern)} ({len(self.pattern) * DT:.1f}s) "
            f"speed={a.speed} m/s max|steer|={max(abs(x) for x in self.pattern):.0f} deg")

    def vs_cb(self, msg):
        self.vs = msg

    def overridden(self):
        v = self.vs
        return (not v.by_wire_enabled) or v.steer_override or v.brake_override or v.throttle_override

    def speed_cmd(self, v_target):
        err = v_target - self.vs.speed_x
        self.integ = max(-2.0, min(2.0, self.integ + err * DT))
        thr = self.a.kp * err + self.a.ki * self.integ
        if thr >= 0:
            return min(thr, self.a.max_throttle), 0.0
        return 0.0, min(0.35, 0.22 - 0.1 * thr) if err < -0.5 else 0.0

    def send(self, steer_deg, thr, brk):
        m = Control()
        m.timestamp = self.get_clock().now().nanoseconds * 1e-9
        m.count = self.count
        m.throttle_cmd = float(thr)
        m.brake_cmd = float(brk)
        m.steering_cmd = math.radians(steer_deg)
        m.gear_cmd = GEAR_DRIVE
        m.turn_signal_cmd = 0
        self.pub.publish(m)
        self.count += 1

    def on_timer(self):
        if self.vs is None:
            return
        if self.phase == "wait_bywire":
            if self.vs.by_wire_enabled:
                self.get_logger().info("by-wire enabled, accelerating to target speed")
                self.phase = "settle"
            return
        if self.phase != "done" and self.overridden():
            self.get_logger().error("by-wire disabled or override detected -> stop commanding")
            self.phase = "done"
            return

        if self.phase == "settle":
            thr, brk = self.speed_cmd(self.a.speed)
            self.send(0.0, thr, brk)
            self.settle_elapsed += DT
            self.settled_t = self.settled_t + DT if abs(self.vs.speed_x - self.a.speed) < 0.3 else 0.0
            if self.settled_t > 2.0 or self.settle_elapsed > 20.0:
                self.get_logger().info(f"speed {self.vs.speed_x:.2f} m/s, playing pattern")
                self.phase = "play"
        elif self.phase == "play":
            thr, brk = self.speed_cmd(self.a.speed)
            self.send(self.pattern[self.k], thr, brk)
            self.k += 1
            if self.k >= len(self.pattern):
                self.get_logger().info("pattern done, stopping")
                self.phase = "stop"
        elif self.phase == "stop":
            self.send(0.0, 0.0, 0.3)
            if self.vs.speed_x < 0.05:
                self.get_logger().info("stopped. done.")
                self.phase = "done"
        elif self.phase == "done":
            raise SystemExit


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pattern", choices=["step", "chirp", "sine"], required=True)
    ap.add_argument("--amps", type=float, nargs="+", default=[30.0], help="steering-wheel deg")
    ap.add_argument("--hold", type=float, default=2.0, help="step hold time [s]")
    ap.add_argument("--repeat", type=int, default=2)
    ap.add_argument("--f0", type=float, default=0.2)
    ap.add_argument("--f1", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--freqs", type=float, nargs="+", default=[0.3, 0.6, 1.0, 1.5])
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--speed", type=float, default=4.0, help="target speed [m/s]")
    ap.add_argument("--max-throttle", type=float, default=0.20)
    ap.add_argument("--kp", type=float, default=0.08)
    ap.add_argument("--ki", type=float, default=0.03)
    ap.add_argument("--ratio-guess", type=float, default=16.0, help="only for the a_y sanity check")
    ap.add_argument("--wheelbase", type=float, default=2.984)
    ap.add_argument("--max-ay", type=float, default=3.0, help="refuse patterns above this steady a_y [m/s^2]")
    a = ap.parse_args()

    amp = max(abs(x) for x in a.amps)
    ay = a.speed ** 2 * math.radians(amp) / a.ratio_guess / a.wheelbase
    if amp > 180 or ay > a.max_ay:
        raise SystemExit(f"refusing: amp {amp} deg at {a.speed} m/s -> ~{ay:.1f} m/s^2 lateral (limit {a.max_ay})")

    rclpy.init()
    node = SteerExcitation(a)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
