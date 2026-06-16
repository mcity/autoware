from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pursuit_control',
            namespace='/mcity',
            executable='pursuit_control',
            name='pursuit_control',
            parameters=[
                # ── Path safety limits ───────────────────────────────────────
                {'max_ey':                      3.0},
                {'max_ephi':                    1.0},
                {'max_curvature':               0.2},

                # ── Cascaded PID: outer loop (velocity → desired acceleration) ──
                {'speed_kp_v':                  1.5},
                {'speed_ki_v':                  0.5},
                {'speed_kd_v':                  0.1},
                # ── Cascaded PID: inner loop (acc error → pedal fraction) ────
                {'speed_kp_a':                  0.10},
                {'speed_ki_a':                  0.02},
                {'speed_kd_a':                  0.0},
                # ── Acceleration estimator low-pass filter ───────────────────
                {'speed_acc_filter_alpha':       0.7},

                # ── Heading / lateral corrections ────────────────────────────
                {'heading_offset':              0.0},
                {'heading_lookahead_points':    3},
                {'lateral_offset':              0.0},

                # ── Trajectory preview window ────────────────────────────────
                {'preview_time':                5.0},
                {'desired_time_resolution':     0.04},

                # ── Trajectory abort thresholds ──────────────────────────────
                {'trajectory_abort_size':       25},
                {'trajectory_loose_abort_size': 75},

                # ── Pure Pursuit gains ───────────────────────────────────────
                # pp_k:     lookahead time [s].  Ld = pp_k * v + pp_Ld_min.
                #           Larger → smoother but slower correction.
                #           Smaller → tighter tracking, may oscillate.
                # pp_Ld_min: minimum lookahead [m] — dominates at low speed.
                # pp_Ld_max: maximum lookahead [m] — caps at high speed.
                # pp_k_ey:   cross-track error gain [1/s].  Adds atan(k_ey*ey/v)
                #            to wheel angle so the car actively returns to path.
                # pp_k_ephi: heading error gain.  Adds k_ephi*ephi (rad) directly
                #            to wheel angle — this is the primary fix for slow
                #            steering return after a lane change.  ephi < 0 when
                #            car is still angled from the lane change → steers back.
                #            Tune: increase if still slow, decrease if oscillating.
                {'pp_k':                        0.5},
                {'pp_Ld_min':                   3.0},
                {'pp_Ld_max':                   8.0},
                {'pp_k_ey':                     0.0},
                {'pp_k_ephi':                   0.0},
                # pp_k_yawdamp: counter-steers proportional to measured yaw rate.
                # Primary fix for slow return after lane change: ephi/ey go to 0
                # once heading catches up, but yaw_rate stays non-zero while the
                # car is still physically rotating and the wheel hasn't unwound.
                # steering_cmd -= k_yawdamp * yaw_rate * STEERING_RATIO
                # Raise if still slow; lower if car oscillates after lane change.
                {'pp_k_yawdamp':                0.0},

                # ── Steering-based speed cap ─────────────────────────────────
                # When measured steering wheel angle >= threshold_deg,
                # desired speed is capped to limit_kmph.
                # After steering returns below threshold, speed ramps back up
                # at ramp_rate m/s² to avoid a sudden throttle spike.
                {'steer_speed_threshold_deg':   90.0},
                {'steer_speed_limit_kmph':      5.0},
                {'steer_speed_ramp_rate':       3.0},

                # ── Steering command slew-rate limit ─────────────────────────
                # Caps how fast the steering command may change [deg/s at the
                # wheel].  Set at or just below the EPS actuator's true max slew
                # (~110 deg/s measured on the Mach-E by-wire) so the command
                # cannot run ahead of the achieved angle — this is the fix for
                # the dead-time limit cycle that diverged at speed.
                {'steer_cmd_rate_dps':          110.0},

                # ── Hard speed cap ───────────────────────────────────────────
                # The ~110 deg/s steering actuator bounds the speed at which the
                # pure-pursuit loop stays stable; above ~2.5 m/s it self-
                # oscillates and diverges (gain × actuator-lag instability).
                # Keep at/below the stable speed until the EPS slew rate is
                # raised.  Set to 25.0 to effectively disable.
                {'max_speed_mps':               2.0},
            ],
            output='screen',
        ),
    ])
