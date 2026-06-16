from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mcity_mache_bywire',
            executable='mcity_mache_bywire',
            name='mcity_mache_bywire',
            parameters=[
                {'max_speed': 20.0},
                {'max_throttle': 0.7},
                {'max_lat_acc': 2.5},
            ],
        ),
    ])
