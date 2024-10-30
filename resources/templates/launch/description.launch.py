from pathlib import Path

import launch
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import launch_ros
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(launch_ros.substitutions.FindPackageShare(package='{package_name}').find('{package_name}'))
    default_model_path = pkg_share / 'urdf/{xacro_wrapper_file}'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {{
                # ParameterValue is required to avoid being interpreted as YAML.
                'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str),
                'use_sim_time': use_sim_time,
            }},
        ],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=str(default_model_path),
            description="Absolute path to the robot's URDF file",
        ),
        use_sim_time_launch_arg,
        robot_state_publisher_node,
    ])
