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
    print(config)
    return LaunchDescription([
        Node(
            package='control',
            namespace='control',
            executable='control_node',
            name='sim',
            parameters=[config]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()