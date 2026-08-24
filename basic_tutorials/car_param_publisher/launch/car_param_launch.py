import launch
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(get_package_share_directory('car_param_publisher'), 'config', 'params.yaml')
    print(config)

    ld = launch.LaunchDescription()

    node = launch_ros.actions.Node(
        package='car_param_publisher',
        executable='param_publisher',
        output='screen',
        name='car_param_node',
        parameters=[config]
    )
    ld.add_action(node)

    return ld