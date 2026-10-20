from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="preview_control",
                namespace="/mcity",
                executable="preview_control",
                parameters=[
                    {
                        "gain_folder": PathJoinSubstitution(
                            [
                                EnvironmentVariable("HOME"),
                                "autoware/src/mcity/preview_control/data/gain/mkz/withoutdelay/",
                            ]
                        )
                    },
                    {"max_ey": 1.5},
                    {"max_ephi": 1.0},
                    {"max_curvature": 0.2},
                    {"speed_ctrl_kp": 1.8},
                    {"speed_ctrl_ki": 0.6},
                    {"heading_offset": -0.03},
                    {"heading_lookahead_points": 6},
                    {"lateral_offset": 0.0},
                    {"trajectory_abort_size": 35},
                    {"trajectory_loose_abort_size": 75},
                ],
            ),
        ]
    )
