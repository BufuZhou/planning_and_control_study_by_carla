import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('control'),
    'param',
    'lat_controller_config.yaml'
    )

    localization_node = Node(
            package='localization',
            namespace='localization',
            executable='localization_node',
            output = 'both'
    )

    planning_node = Node(
            package='planning',
            namespace='planning',
            executable='planning_node',
            output = 'both'
    )

    control_node = Node(
            package='control',
            namespace='control',
            executable='control_node',
            parameters=[config],
            output = 'both'
    )

    return LaunchDescription([
        localization_node,
        planning_node,
        control_node
    ])

if __name__ == '__main__':
    generate_launch_description()