from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg = FindPackageShare("my_robot_description")

    robot_desc = Command([
        "cat ",
        PathJoinSubstitution([pkg, "urdf", "robot.urdf"])
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_desc},
            PathJoinSubstitution([pkg, "config", "controllers.yaml"]),
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cmdvel_diffdrive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        control_node,
        jsb_spawner,
        ctrl_spawner,
    ])
