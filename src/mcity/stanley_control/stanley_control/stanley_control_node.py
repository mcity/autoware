"""
stanley_control_node.py

ROS2 node: Stanley lateral control + cascaded PID speed control (Mach-E).

Subscribes  (same topics as preview_control):
  /mcity/cav_pose          — geometry_msgs/PoseWithCovarianceStamped
  /mcity/vehicle_state     — mcity_msgs/VehicleState
  /mcity/vehicle_planning  — mcity_msgs/VehiclePlanning

Publishes   (new topic name):
  /mcity/stanley_control/vehicle_control  — mcity_msgs/Control

Timer: 50 Hz (20 ms), identical to preview_control.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcity_msgs.msg import Control, VehicleState, VehiclePlanning

from .path_process import PathProcessor, quaternion_to_yaw
from .stanley_controller import StanleyController
from .speed_control import (
    SpeedController,
    GEAR_DRIVE, GEAR_PARK,
    ESTOP_NONE, ESTOP_LOW, ESTOP_MEDIUM, ESTOP_HIGH,
)

CONTROL_FREQ_HZ = 50          # Hz
TIMER_PERIOD_S  = 1.0 / CONTROL_FREQ_HZ   # 0.02 s


class StanleyControlNode(Node):

    def __init__(self):
        super().__init__('stanley_control')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('slope_folder',                  '')
        self.declare_parameter('max_ey',                        1.5)
        self.declare_parameter('max_ephi',                      1.0)
        self.declare_parameter('max_curvature',                 0.2)
        self.declare_parameter('heading_offset',                0.0)
        self.declare_parameter('heading_lookahead_points',      3)
        self.declare_parameter('lateral_offset',                0.0)
        self.declare_parameter('preview_time',                  5.0)
        self.declare_parameter('desired_time_resolution',       0.04)
        self.declare_parameter('trajectory_abort_size',         25)
        self.declare_parameter('trajectory_loose_abort_size',   75)
        # Stanley steering gains
        self.declare_parameter('stanley_k',                     0.5)
        self.declare_parameter('stanley_k_soft',                1.0)
        # Cascaded PID — outer loop (velocity → desired acceleration)
        self.declare_parameter('speed_kp_v',                    1.5)
        self.declare_parameter('speed_ki_v',                    0.5)
        self.declare_parameter('speed_kd_v',                    0.1)
        # Cascaded PID — inner loop (acc error → pedal fraction)
        self.declare_parameter('speed_kp_a',                    0.10)
        self.declare_parameter('speed_ki_a',                    0.02)
        self.declare_parameter('speed_kd_a',                    0.0)
        # Acceleration estimator low-pass filter coefficient [0–1)
        self.declare_parameter('speed_acc_filter_alpha',        0.7)

        max_ey         = self.get_parameter('max_ey').value
        max_ephi       = self.get_parameter('max_ephi').value
        heading_offset = self.get_parameter('heading_offset').value
        lookahead_pts  = self.get_parameter('heading_lookahead_points').value
        lateral_offset = self.get_parameter('lateral_offset').value
        preview_time   = self.get_parameter('preview_time').value
        desired_dt     = self.get_parameter('desired_time_resolution').value
        stanley_k      = self.get_parameter('stanley_k').value
        stanley_k_soft = self.get_parameter('stanley_k_soft').value

        self._trajectory_abort_size       = self.get_parameter('trajectory_abort_size').value
        self._trajectory_loose_abort_size = self.get_parameter('trajectory_loose_abort_size').value
        self._preview_time = preview_time
        self._desired_dt   = desired_dt
        self._max_ey       = max_ey
        self._max_ephi     = max_ephi

        # ── Internal state ───────────────────────────────────────────────────
        self._pos_x   = 0.0
        self._pos_y   = 0.0
        self._pos_z   = 0.0
        self._qx      = 0.0
        self._qy      = 0.0
        self._qz      = 0.0
        self._qw      = 1.0

        self._speed_x             = 0.0
        self._yaw_rate            = 0.0
        self._steering_wheel_angle = 0.0
        self._gear_pos            = GEAR_DRIVE
        self._by_wire_enabled     = False
        self._brake_state         = 0.0
        self._throttle_state      = 0.0

        self._p2c_timestamp       = 0.0
        self._p2c_go              = 0
        self._p2c_estop           = ESTOP_NONE
        self._p2c_acc_d           = 0.0
        self._p2c_time_resolution = 0.04
        self._p2c_x_vec           = []
        self._p2c_y_vec           = []
        self._p2c_vd_vec          = []
        self._p2c_ori_vec         = []

        self._stop_count = 0

        # ── Sub-modules ──────────────────────────────────────────────────────
        self._path_proc = PathProcessor(
            max_ey=max_ey,
            max_ephi=max_ephi,
            heading_offset=heading_offset,
            heading_lookahead_points=lookahead_pts,
            lateral_offset=lateral_offset,
        )

        self._stanley = StanleyController(k=stanley_k, k_soft=stanley_k_soft)

        self._speed_ctrl = SpeedController(
            kp_v=self.get_parameter('speed_kp_v').value,
            ki_v=self.get_parameter('speed_ki_v').value,
            kd_v=self.get_parameter('speed_kd_v').value,
            kp_a=self.get_parameter('speed_kp_a').value,
            ki_a=self.get_parameter('speed_ki_a').value,
            kd_a=self.get_parameter('speed_kd_a').value,
            acc_filter_alpha=self.get_parameter('speed_acc_filter_alpha').value,
            frequency=CONTROL_FREQ_HZ,
        )

        # ── ROS I/O ──────────────────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(
            Control,
            '/mcity/preview_control/vehicle_control',
            10,
        )

        self._sub_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/mcity/cav_pose',
            self._pose_cb,
            10,
        )
        self._sub_state = self.create_subscription(
            VehicleState,
            '/mcity/vehicle_state',
            self._vehicle_state_cb,
            10,
        )
        self._sub_path = self.create_subscription(
            VehiclePlanning,
            '/mcity/vehicle_planning',
            self._planning_cb,
            10,
        )

        self._timer = self.create_timer(TIMER_PERIOD_S, self._on_timer)
        self.get_logger().info('stanley_control node started')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y
        self._pos_z = msg.pose.pose.position.z
        self._qx    = msg.pose.pose.orientation.x
        self._qy    = msg.pose.pose.orientation.y
        self._qz    = msg.pose.pose.orientation.z
        self._qw    = msg.pose.pose.orientation.w

    def _vehicle_state_cb(self, msg: VehicleState):
        self._speed_x              = msg.speed_x
        self._yaw_rate             = msg.yaw_rate
        self._steering_wheel_angle = msg.steer_state
        self._gear_pos             = msg.gear_pos
        self._by_wire_enabled      = msg.by_wire_enabled
        self._brake_state          = msg.brake_state
        self._throttle_state       = msg.throttle_state

    def _planning_cb(self, msg: VehiclePlanning):
        if not msg.x_vector:
            return

        self._p2c_timestamp       = msg.timestamp
        self._p2c_time_resolution = msg.time_resolution
        self._p2c_estop           = msg.estop
        self._p2c_go              = msg.go

        self._p2c_x_vec   = list(msg.x_vector)
        self._p2c_y_vec   = list(msg.y_vector)
        self._p2c_vd_vec  = list(msg.vd_vector)
        self._p2c_ori_vec = list(msg.ori_vector)

        self._path_proc.process_path(
            self._p2c_x_vec,
            self._p2c_y_vec,
            self._p2c_vd_vec,
            self._p2c_ori_vec,
            self._p2c_time_resolution,
            self._preview_time,
            self._desired_dt,
        )

    # ── Control timer (50 Hz) ────────────────────────────────────────────────

    def _on_timer(self):
        cmd = Control()
        cmd.timestamp     = self.get_clock().now().nanoseconds * 1e-9
        cmd.gear_cmd      = GEAR_DRIVE
        cmd.turn_signal_cmd = 0

        # ── Guard: go flag ───────────────────────────────────────────────────
        if self._p2c_go == 0:
            throttle, brake, gear = self._speed_ctrl.set_stop(self._speed_x)
            self._publish(cmd, 0.0, throttle, brake, gear)
            self.get_logger().info(
                'go=0, stopping', throttle_duration_sec=1.0)
            return

        # ── Guard: stale planning ────────────────────────────────────────────
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._p2c_timestamp > 1.0:
            throttle, brake, gear = self._speed_ctrl.set_stop(self._speed_x)
            self._publish(cmd, 0.0, throttle, brake, gear)
            self.get_logger().warn(
                'Planning stale (>1 s), stopping',
                throttle_duration_sec=1.0,
            )
            return

        # ── Guard: trajectory too short ──────────────────────────────────────
        remaining = self._path_proc.remaining_size()
        if remaining < self._trajectory_abort_size:
            throttle, brake, gear = self._speed_ctrl.set_stop(self._speed_x)
            self._publish(cmd, 0.0, throttle, brake, gear)
            self.get_logger().warn(
                f'Trajectory too short ({remaining}), stopping',
                throttle_duration_sec=1.0,
            )
            return

        # ── Guard: near end + stopped ────────────────────────────────────────
        if remaining < self._trajectory_loose_abort_size and self._speed_x == 0.0:
            if self._stop_count > 100:
                throttle, brake, gear = self._speed_ctrl.set_stop(self._speed_x)
                self._publish(cmd, 0.0, throttle, brake, gear)
                return
            self._stop_count += 1
        else:
            self._stop_count = 0

        # ── Step 1: path errors ──────────────────────────────────────────────
        self._path_proc.run(
            self._pos_x, self._pos_y,
            self._qx, self._qy, self._qz, self._qw,
            self._speed_x,
        )
        ey   = self._path_proc.ey
        ephi = self._path_proc.ephi
        vd   = self._path_proc.vd
        slope = self._path_proc.slope

        self.get_logger().info(
            f'idx={self._path_proc.closest_index} '
            f'ey={ey:.3f} ephi={ephi:.3f} '
            f'vd={vd:.2f} vc={self._speed_x:.2f}',
            throttle_duration_sec=0.2,
        )

        # ── Step 2: in-path check ────────────────────────────────────────────
        in_path = abs(ey) < self._max_ey and abs(ephi) < self._max_ephi
        if not in_path and self._p2c_estop < ESTOP_HIGH:
            self._p2c_estop = ESTOP_HIGH
            self.get_logger().warn(
                f'Out of path (ey={ey:.2f}, ephi={ephi:.2f}), estop HIGH',
                throttle_duration_sec=0.5,
            )

        # ── Step 3: Stanley steering ─────────────────────────────────────────
        yaw = quaternion_to_yaw(self._qx, self._qy, self._qz, self._qw)
        steering_cmd = 0.0

        if self._speed_x > 0.001 and len(self._path_proc.x_vec) >= 2:
            steering_cmd = self._stanley.compute_steering(
                pos_x=self._pos_x,
                pos_y=self._pos_y,
                yaw=yaw,
                speed_x=self._speed_x,
                x_vec=self._path_proc.x_vec,
                y_vec=self._path_proc.y_vec,
                ori_vec=self._path_proc.ori_vec,
                heading_offset=self._path_proc.heading_offset,
            )

        # ── Step 4: speed control ────────────────────────────────────────────
        throttle, brake, gear = self._speed_ctrl.run(
            vd=vd,
            speed_x=self._speed_x,
            slope=slope,
            gear_pos=self._gear_pos,
            estop=self._p2c_estop,
            acc_d=self._p2c_acc_d,
        )

        self._publish(cmd, steering_cmd, throttle, brake, gear)

    # ── publish helper ───────────────────────────────────────────────────────

    def _publish(
        self,
        cmd: Control,
        steering: float,
        throttle: float,
        brake: float,
        gear: int,
    ):
        cmd.steering_cmd  = float(steering)
        cmd.throttle_cmd  = float(throttle)
        cmd.brake_cmd     = float(brake)
        cmd.gear_cmd      = int(gear)
        self._pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
