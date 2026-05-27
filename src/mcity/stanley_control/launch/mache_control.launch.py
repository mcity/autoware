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
                # stanley_k_soft: min effective speed [m/s] in the CTE term
                #                 — prevents large steering at near-zero speed.
                {'stanley_k':                   0.5},
                {'stanley_k_soft':              1.0},
            ],
            output='screen',
        ),
    ])
