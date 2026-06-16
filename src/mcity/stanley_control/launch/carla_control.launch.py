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
                {
                    'slope_folder': PathJoinSubstitution([
                        EnvironmentVariable('HOME'),
                        'autoware/src/mcity/preview_control/data/slope/DIT.txt',
                    ])
                },
                {'max_ey':                      1.5},
                {'max_ephi':                    1.0},
                {'max_curvature':               0.2},
                {'speed_ctrl_kp':               1.8},
                {'speed_ctrl_ki':               0.8},
                {'heading_offset':             -0.03},
                {'heading_lookahead_points':    4},
                {'lateral_offset':              0.0},
                {'preview_time':                5.0},
                {'desired_time_resolution':     0.04},
                {'trajectory_abort_size':       25},
                {'trajectory_loose_abort_size': 75},
                {'stanley_k':                   0.5},
                {'stanley_k_soft':              1.0},
                {'stanley_k_yaw':               0.5},
            ],
            output='screen',
        ),
    ])
