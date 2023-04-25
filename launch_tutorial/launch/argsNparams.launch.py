import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    ld = LaunchDescription()

    test_arg = DeclareLaunchArgument(
        name='test_arg1',
        default_value="tt",
        description='This is the test argument'
    )
    ld.add_action(test_arg)

    test_arg_val = LaunchConfiguration('test_arg1',default='tt')
    
    print_arg = LogInfo(msg=test_arg_val)
    ld.add_action(print_arg)

    return ld


# usage
# ros2 launch launch_tutorial argsNparams.launch.py test_arg1:='cc'
# It should print out 'cc' in the terminal
