from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    description_file = PathJoinSubstitution(
        [FindPackageShare("rebuild_posital"), "description", "posital_inclinometer.urdf.xacro"]
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("rebuild_posital"), "config", "posital_controllers.yaml"]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_file,
                " can_interface:=",
                can_interface,
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="pcan1"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controller_config],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["posital_pitch_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]
    )
