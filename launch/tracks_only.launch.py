from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("can_driver")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([share, "launch", "can_driver.launch.py"])),
            launch_arguments={
                "config_file": LaunchConfiguration(
                    "config_file", default=PathJoinSubstitution(
                        [share, "config", "can_driver_tracks_only.yaml"])),
                "controllers_file": LaunchConfiguration(
                    "controllers_file", default=PathJoinSubstitution(
                        [share, "config", "ros2_controllers_tracks_only.yaml"])),
                "use_ros2_control": "true",
                "start_controllers": "joint_state_broadcaster wheel_controller",
            }.items(),
        ),
    ])
