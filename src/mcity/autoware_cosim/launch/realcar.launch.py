from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Create nodes for all executables
    autoware_vehicle_plugin_node = Node(
        package='autoware_cosim',
        executable='autoware_vehicle_plugin',
        name='autoware_vehicle_plugin',
        output='screen',
        parameters=[{
            'control_cav': False,
            'cosim_controlled_vehicle_keys': ['terasim_actor_info','cav_1_info','cav_2_info'],
        }]
    )

    autoware_tls_plugin_node = Node(
        package='autoware_cosim',
        executable='autoware_tls_plugin',
        name='autoware_tls_plugin',
        output='screen'
    )

    autoware_dummy_grid_node = Node(
        package='autoware_cosim',
        executable='autoware_dummy_grid',
        name='autoware_dummy_grid',
        output='screen'
    )

    autoware_vehicle_report_node = Node(
        package='autoware_cosim',
        executable='autoware_vehicle_report',
        name='autoware_vehicle_report',
        output='screen'
    )

    autoware_planning_node = Node(
        package='autoware_cosim',
        executable='autoware_planning',
        name='autoware_planning',
        output='screen'
    )

    gnss_node = Node(
        package='gnss_decoder',
        executable='gnss_decoder',
        name='gnss_decoder',
        output='screen'
    )

    return LaunchDescription([
        autoware_vehicle_plugin_node,
        autoware_vehicle_report_node,
        autoware_tls_plugin_node,
        autoware_dummy_grid_node,
        autoware_planning_node,
        gnss_node
    ]) 
