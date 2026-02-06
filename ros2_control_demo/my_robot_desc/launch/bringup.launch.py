from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    if_simulation = LaunchConfiguration("simulation")

    # robot_description (publish on /robot_description)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("my_robot_desc"), "urdf", "robot.urdf.xacro"]
            ),
            " ",
            "simulation:=", if_simulation,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # controllers yaml
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("my_robot_desc"), "config", "controllers.yaml"]
    )

    # ros2_control_node: ONLY load controllers.yaml (official style)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
    )

    # robot_state_publisher: publish /robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # spawners (official style)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    cmdvel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cmdvel_diffdrive_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", robot_controllers,
        ],
        output="screen",
    )

    # delay cmdvel controller until JSB spawner exits
    delay_cmdvel_spawner_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[cmdvel_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_cmdvel_spawner_after_jsb,
        ]
    )
