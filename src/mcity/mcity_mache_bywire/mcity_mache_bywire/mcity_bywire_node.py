#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from mcity_msgs.msg import Control, VehicleState

# New unified ds_dbw_msgs package (replaces dataspeed_dbw_msgs / dbw_ford_msgs / dbw_fca_msgs)
from ds_dbw_msgs.msg import (
    ThrottleCmd, ThrottleReport,
    BrakeCmd, BrakeReport,
    SteeringCmd, SteeringReport,
    GearCmd, GearReport,
    MiscCmd,
    Gear, TurnSignal,WheelSpeeds
)

# Vehicle-specific messages not yet in ds_dbw_msgs
from dbw_ford_msgs.msg import ThrottleInfoReport
#from dbw_fca_msgs.msg import WheelSpeedReport

# Physical constants
MAX_STEERING_ANGLE = 2.5 * math.pi   # rad
MIN_STEERING_ANGLE = -2.5 * math.pi  # rad
MAX_BRAKE = 1.00
WHEEL_RADIUS = 0.673 / 2.0           # m  (WHEEL_DIAMETER / 2)

# Gear values (mirrors Gear.msg constants)
GEAR_NONE = 0
GEAR_PARK = 1
GEAR_REVERSE = 2
GEAR_NEUTRAL = 3
GEAR_DRIVE = 4
GEAR_LOW = 5


class McityBywire(Node):
    def __init__(self):
        super().__init__('mcity_bywire')

        self.declare_parameter('max_speed', 17.8333)
        self.declare_parameter('max_throttle', 0.45)
        self.declare_parameter('max_lat_acc', 2.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.max_lat_acc = self.get_parameter('max_lat_acc').value

        # --- Publishers ---
        self.pub_throttle  = self.create_publisher(ThrottleCmd,    '/vehicle/throttle/cmd',   10)
        self.pub_brake     = self.create_publisher(BrakeCmd,       '/vehicle/brake/cmd',      10)
        self.pub_steering  = self.create_publisher(SteeringCmd,    '/vehicle/steering/cmd',   10)
        self.pub_gear      = self.create_publisher(GearCmd,        '/vehicle/gear/cmd',       10)
        self.pub_misc      = self.create_publisher(MiscCmd,        '/vehicle/misc/cmd',       10)
        self.pub_veh_state = self.create_publisher(VehicleState,   '/mcity/vehicle_state',    10)

        # --- Subscribers ---
        self.create_subscription(ThrottleReport,   '/vehicle/throttle/report',      self._throttle_rept_cb,      10)
        self.create_subscription(ThrottleInfoReport, '/vehicle/throttle/info_report', self._throttle_info_rept_cb, 10)
        self.create_subscription(BrakeReport,      '/vehicle/brake/report',         self._brake_rept_cb,         10)
        self.create_subscription(SteeringReport,   '/vehicle/steering/report',      self._steer_rept_cb,         10)
        self.create_subscription(GearReport,       '/vehicle/gear/report',          self._gear_rept_cb,          10)
        self.create_subscription(WheelSpeeds, '/vehicle/wheel_speeds',   self._wheelspeed_rept_cb,    10)
        self.create_subscription(Bool,             '/vehicle/dbw_enabled',          self._sys_enable_cb,         10)
        self.create_subscription(Control,          '/mcity/vehicle_control',        self._cmd_cb,                10)

        # 50 Hz timer (20 ms)
        self.timer = self.create_timer(0.02, self._on_timer)

        # Shared vehicle state published to /mcity/vehicle_state
        self.vs_msg = VehicleState()
        self.is_cmd_received = False

        # Current incoming command values
        self.cmd_throttle    = 0.0
        self.cmd_brake       = 0.0
        self.cmd_steering    = 0.0
        self.cmd_gear        = GEAR_NONE
        self.cmd_turn_signal = TurnSignal.NONE

        # --- Pre-built outgoing command messages ---

        # ThrottleCmd: CMD_PERCENT (=14) for ds_dbw_msgs
        self.throttle_msg = ThrottleCmd()
        self.throttle_msg.enable   = True
        self.throttle_msg.ignore   = False
        self.throttle_msg.clear    = False
        self.throttle_msg.cmd_type = ThrottleCmd.CMD_PERCENT  # 14
        self.throttle_msg.cmd      = 0.0

        # BrakeCmd: CMD_PERCENT (=14) for ds_dbw_msgs
        self.brake_msg = BrakeCmd()
        self.brake_msg.enable   = True
        self.brake_msg.ignore   = False
        self.brake_msg.clear    = False
        self.brake_msg.cmd_type = BrakeCmd.CMD_PERCENT  # 14
        self.brake_msg.cmd      = 0.0

        # SteeringCmd: CMD_ANGLE (=2) for ds_dbw_msgs -MKZ
        # SteeringCmd: CMD_YAW_RATE (=4) for ds_dbw_msgs -MachE
        self.steering_msg = SteeringCmd()
        self.steering_msg.enable   = True
        self.steering_msg.ignore   = False
        self.steering_msg.clear    = False
        self.steering_msg.cmd_type = SteeringCmd.CMD_ANGLE   # 2
        self.steering_msg.cmd      = 0.0
        # Steering rate limit. ds_dbw SteeringCmd.cmd_rate is in DEG/S
        # (range 0-1016; 0 = firmware default, INFINITY = unlimited).
        # The previous value 1.75*pi was written as if rad/s — it evaluated to
        # ~5.5, an invalid/tiny rate that the firmware ignored, so the wheel ran
        # at the conservative firmware default (~110 deg/s) and capped the speed
        # at which the lateral controller stays stable.  300 deg/s is a
        # deliberate, still-conservative limit; raise toward the EPS hardware
        # max once validated on the vehicle.
        self.steering_msg.cmd_rate = 300.0  # deg/s

        # GearCmd
        self.gear_msg = GearCmd()
        self.gear= Gear()

        self.gear_msg.cmd.value = self.gear.NONE 
        self.gear_msg.return_to_park = False

        # MiscCmd — turn signal lives here in ds_dbw_msgs
        self.misc_msg = MiscCmd()
        #self.misc_msg.turn_signal.value = TurnSignal.NONE

    # ------------------------------------------------------------------
    # Timer callback (50 Hz)
    # ------------------------------------------------------------------

    def _on_timer(self):
        if self.is_cmd_received:
            self._check()
            self._publish_cmd()
            self.is_cmd_received = False
        self._publish_veh_state()

    # ------------------------------------------------------------------
    # Command output
    # ------------------------------------------------------------------
    def _publish_cmd(self):
        if not self.vs_msg.by_wire_enabled:
            self.get_logger().warn(
                'Drive command received but drive by wire is not enabled',
                throttle_duration_sec=1.0)
            return

        # Gear shift — only when stopped and braking
        self.gear_msg.cmd.value = self.cmd_gear
        if self.cmd_gear != self.vs_msg.gear_pos and self.vs_msg.speed_x == 0.0:
            self.cmd_brake    = 0.0
            self.cmd_throttle = 0.0
            if self.vs_msg.brake_state > 0:
                self.pub_gear.publish(self.gear_msg)
                self.get_logger().info('Shift gear')

        # Throttle
        self.throttle_msg.cmd = self.cmd_throttle*100
        self.pub_throttle.publish(self.throttle_msg)

        # Brake
        self.brake_msg.cmd = self.cmd_brake*100
        self.pub_brake.publish(self.brake_msg)

        # Steering
        self.steering_msg.cmd = self.cmd_steering * (180/math.pi)
        self.pub_steering.publish(self.steering_msg)

        # Turn signal via MiscCmd
        #self.misc_msg.turn_signal.value = self.cmd_turn_signal
        if self.cmd_turn_signal != TurnSignal.NONE:
            self.pub_misc.publish(self.misc_msg)

    def _publish_veh_state(self):
        self.vs_msg.timestamp = self.get_clock().now().nanoseconds * 1e-9
        self.pub_veh_state.publish(self.vs_msg)

    # ------------------------------------------------------------------
    # Safety checks
    # ------------------------------------------------------------------

    def _check(self):
        # Rule 1: Throttle in [0, max_throttle]
        if self.cmd_throttle > self.max_throttle + 0.01:
            self.get_logger().warn(
                f'Violate rule 1: throttle {self.cmd_throttle:.4f} > max {self.max_throttle:.4f}, clamped')
            self.cmd_throttle = self.max_throttle
        if self.cmd_throttle < 0:
            self.get_logger().warn(
                f'Violate rule 1: throttle {self.cmd_throttle:.4f} < 0, set to 0')
            self.cmd_throttle = 0.0

        # Rule 2: Brake in [0, MAX_BRAKE]
        if self.cmd_brake > MAX_BRAKE + 0.01:
            self.get_logger().warn(
                f'Violate rule 2: brake {self.cmd_brake:.4f} > max {MAX_BRAKE:.4f}, clamped')
            self.cmd_brake    = MAX_BRAKE
            self.cmd_throttle = 0.0
        if self.cmd_brake < 0:
            self.get_logger().warn(
                f'Violate rule 2: brake {self.cmd_brake:.4f} < 0, set to 0.0')
            self.cmd_brake    = 0.0
            self.cmd_throttle = 0.0

        # Rule 3: Steering in [MIN_STEERING_ANGLE, MAX_STEERING_ANGLE]
        if self.cmd_steering > MAX_STEERING_ANGLE:
            self.get_logger().warn(
                f'Violate rule 3: steering {self.cmd_steering:.4f} > max {MAX_STEERING_ANGLE:.4f}, clamped')
            self.cmd_steering = MAX_STEERING_ANGLE
        if self.cmd_steering < MIN_STEERING_ANGLE:
            self.get_logger().warn(
                f'Violate rule 3: steering {self.cmd_steering:.4f} < min {MIN_STEERING_ANGLE:.4f}, clamped')
            self.cmd_steering = MIN_STEERING_ANGLE

        # Rule 4: Speed limit
        if self.vs_msg.speed_x > self.max_speed + 0.2:
            self.get_logger().warn(
                f'Violate rule 4: speed {self.vs_msg.speed_x:.4f} > {self.max_speed:.4f} m/s, throttle zeroed')
            self.cmd_throttle = 15.0

        # Rule 5: No gear shift while moving
        if (self.vs_msg.speed_x > 0 and
                self.cmd_gear != self.vs_msg.gear_pos and
                self.cmd_gear != GEAR_NONE):
            self.get_logger().warn(
                f'Violate rule 5: gear shift attempted at speed {self.vs_msg.speed_x:.4f} m/s')
            self.cmd_gear = GEAR_NONE

        # Rule 6: Lateral acceleration limit
        if abs(self.vs_msg.acc_y) > self.max_lat_acc:
            self.get_logger().warn(
                f'Violate rule 6: a_y {self.vs_msg.acc_y:.4f} > {self.max_lat_acc:.4f} m/s², throttle zeroed')
            self.cmd_throttle = 15.0

        # Final: disable throttle whenever brake is applied
        if self.cmd_brake > 0.0:
            self.cmd_throttle = 0.0

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Control):
        self.cmd_throttle    = msg.throttle_cmd
        self.cmd_brake       = msg.brake_cmd
        self.cmd_steering    = msg.steering_cmd
        self.cmd_gear        = msg.gear_cmd
        self.cmd_turn_signal = msg.turn_signal_cmd
        self.is_cmd_received = True

    def _sys_enable_cb(self, msg: Bool):
        self.vs_msg.by_wire_enabled = msg.data

    def _throttle_rept_cb(self, msg: ThrottleReport):
        # ds_dbw_msgs ThrottleReport uses percent_* instead of pedal_*
        # override field is now override_active; driver field removed
        self.vs_msg.throttle_cmd      = self.float_check(msg.percent_cmd)
        self.vs_msg.throttle_input    = self.float_check(msg.percent_input)
        self.vs_msg.throttle_state    = self.float_check(msg.percent_output)
        self.vs_msg.throttle_enabled  = msg.enabled
        self.vs_msg.throttle_override = msg.override_active
        self.vs_msg.throttle_timeout  = msg.timeout

    def _throttle_info_rept_cb(self, msg: ThrottleInfoReport):
        self.vs_msg.engine_rpm = msg.engine_rpm

    def _brake_rept_cb(self, msg: BrakeReport):
        # ds_dbw_msgs BrakeReport uses percent_* instead of pedal_*
        # override field is now override_active; driver field removed
        self.vs_msg.brake_cmd        = 0.0
        self.vs_msg.brake_input      = 0.0
        self.vs_msg.brake_state      = max(0.0, min(100.0, (msg.torque_input/16376.0) *100))
        self.vs_msg.brake_torq_cmd   = 0.0
        self.vs_msg.brake_torq_input = 0.0
        self.vs_msg.brake_torq_state = 0.0
        self.vs_msg.brake_enabled    = msg.enabled
        self.vs_msg.brake_override   = msg.override_active
        self.vs_msg.brake_timeout    = msg.timeout

    def _steer_rept_cb(self, msg: SteeringReport):
        # ds_dbw_msgs SteeringReport: torque field renamed; speed field removed
        # Vehicle speed is now derived from WheelSpeeds
        self.vs_msg.steer_state    = msg.steering_wheel_angle * (math.pi/180)
        self.vs_msg.steer_torque   = msg.steering_column_torque  # renamed from steering_wheel_torque
        self.vs_msg.steer_enabled  = msg.enabled
        self.vs_msg.steer_override = msg.override_active
        self.vs_msg.steer_timeout  = msg.timeout

    def _gear_rept_cb(self, msg: GearReport):
        # ds_dbw_msgs GearReport: field renamed from 'state' to 'gear'
        self.vs_msg.gear_pos = msg.gear.value

    def _wheelspeed_rept_cb(self, msg: WheelSpeeds):
        self.vs_msg.wheel_v_front_left  = msg.front_left
        self.vs_msg.wheel_v_front_right = msg.front_right
        self.vs_msg.wheel_v_rear_left   = msg.rear_left
        self.vs_msg.wheel_v_rear_right  = msg.rear_right
        # Derive longitudinal vehicle speed; SteeringReport no longer carries speed in ds_dbw_msgs
        avg_rad_s = (msg.front_left + msg.front_right +
                     msg.rear_left  + msg.rear_right) / 4.0
        self.vs_msg.speed_x = avg_rad_s * WHEEL_RADIUS

    def float_check(slef, float_msg):
        return float_msg if isinstance(float_msg,float) else 0.0

    def bool_check(slef, bool_msg):
        return bool_msg if isinstance(bool_msg,bool) else False

def main(args=None):
    rclpy.init(args=args)
    node = McityBywire()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
