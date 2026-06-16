from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stanley_control',
            namespace='/mcity',
            executable='stanley_control',
            name='stanley_control',
            parameters=[
                # ── Path safety limits ───────────────────────────────────────
                {'max_ey':                      1.5},
                {'max_ephi':                    1.0},
                {'max_curvature':               0.2},

                # ── Cascaded PID: outer loop (velocity → desired acceleration) ──
                # kp_v: 1 m/s speed error → kp_v m/s² acceleration demand.
                # ki_v: eliminates steady-state speed error (wind-up clamped).
                # kd_v: derivative on measurement — dampen overshoot.
                {'speed_kp_v':                  1.5},
                {'speed_ki_v':                  0.5},
                {'speed_kd_v':                  0.1},
                # ── Cascaded PID: inner loop (acc error → pedal fraction) ────
                # kp_a: 1 m/s² acc error → kp_a pedal fraction change.
                # ki_a: trims steady-state acc error (wind-up clamped).
                # kd_a: set to 0 until outer loop is stable, then tune.
                {'speed_kp_a':                  0.10},
                {'speed_ki_a':                  0.02},
                {'speed_kd_a':                  0.0},
                # ── Acceleration estimator low-pass filter ───────────────────
                # alpha closer to 1 = more smoothing, more lag.
                # alpha closer to 0 = less lag, more noise in inner loop.
                {'speed_acc_filter_alpha':       0.7},

                # ── Heading / lateral corrections ────────────────────────────
                # heading_offset: bias between RTK antenna and CoM heading.
                # Measure by driving straight and checking steady-state ephi.
                {'heading_offset':              0.0},
                {'heading_lookahead_points':    3},
                {'lateral_offset':              0.0},

                # ── Trajectory preview window ────────────────────────────────
                {'preview_time':                5.0},
                {'desired_time_resolution':     0.04},

                # ── Trajectory abort thresholds ──────────────────────────────
                {'trajectory_abort_size':       25},
                {'trajectory_loose_abort_size': 75},

                # ── Stanley gains ────────────────────────────────────────────
                # stanley_k:      increase for tighter tracking; decrease if
                #                 oscillating. Tune on vehicle, start at 0.5.
                # stanley_k_soft: added to speed in the CTE-term denominator
                #                 (atan(k*e / (k_soft + v))). The low-speed
                #                 cross-track gain is k/k_soft — keep it ≲1 to
                #                 stop the steering oscillating near a standstill.
                {'stanley_k':                   2.0},
                {'stanley_k_soft':              2.0},
                # stanley_k_yaw: yaw-rate damping gain [s]. Damps the heading
                #                feedback. NOTE: the vehicle's reported yaw_rate
                #                is unpopulated, so the node derives yaw rate
                #                kinematically from the measured steering. Raise
                #                if it still oscillates; lower (toward 0) if it
                #                feels sluggish or over-damped on curves.
                {'stanley_k_yaw':               0.3},
                # Rate limiter: max steering-wheel angle change per second [rad/s].
                # 1.5 was far too slow: the limiter wound up to full lock and
                # took seconds to unwind, self-sustaining a steering limit cycle.
                {'steer_rate_limit':            4.0},
                # Anti-wind-up: max lead [rad] the command may have over the
                # measured wheel. Caps the limiter so it cannot run to full lock
                # ahead of the actuator. Lower if steering still overshoots.
                {'steer_windup_band':           0.6},
            ],
            output='screen',
        ),
    ])
