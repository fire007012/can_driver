from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_share = FindPackageShare("can_driver")
    config_file = LaunchConfiguration("config_file")
    controllers_file = LaunchConfiguration("controllers_file")
    robot_description_file = LaunchConfiguration("robot_description_file")
    start_controllers = LaunchConfiguration("start_controllers")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    standalone = Node(
        package="can_driver",
        executable="can_driver_node",
        name="can_driver_node",
        output="screen",
        parameters=[{"config_file": config_file}],
        condition=UnlessCondition(use_ros2_control),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": Command(["cat ", robot_description_file])},
            controllers_file,
        ],
        condition=IfCondition(use_ros2_control),
    )

    controller_names = start_controllers.perform(context).split()
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            name=f"{controller_name}_spawner",
            output="screen",
            arguments=[controller_name, "--controller-manager", "/controller_manager"],
            condition=IfCondition(use_ros2_control),
        )
        for controller_name in controller_names
    ]

    return [standalone, control_node, *spawners]


def generate_launch_description():
    share = FindPackageShare("can_driver")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([share, "config", "can_driver.yaml"]),
            description="can_driver YAML configuration"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([share, "config", "ros2_controllers.yaml"]),
            description="ros2_control controller parameters"),
        DeclareLaunchArgument(
            "robot_description_file",
            default_value=PathJoinSubstitution([share, "config", "can_driver.ros2_control.urdf"]),
            description="URDF containing the can_driver ros2_control component"),
        DeclareLaunchArgument("use_ros2_control", default_value="false"),
        DeclareLaunchArgument(
            "start_controllers",
            default_value="joint_state_broadcaster arm_controller wheel_controller"),
        SetEnvironmentVariable(
            "ACTUATOR_TOOL_PATH",
            PathJoinSubstitution([share, "lib", "innfos-cpp-sdk", "tools", "logtool", "linux_x86_64"])),
        OpaqueFunction(function=launch_setup),
    ])
