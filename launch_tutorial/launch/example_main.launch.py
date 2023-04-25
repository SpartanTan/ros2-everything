from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    PythonLaunchDescriptionSource()
    PathJoinSubstitution(FindPackageShare('launch_tutorial'), 'example_substitutions.launch.py')
    