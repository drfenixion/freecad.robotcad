from pathlib import Path

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = Path(launch_ros.substitutions.FindPackageShare(package='{package_name}').find('{package_name}'))
    default_model_path = pkg_share / 'urdf/{urdf_file}'
    default_rviz_config_path = pkg_share / 'rviz/robot_description.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('{package_name}'),
                'launch',
                'description.launch.py',
            ]),
        ]),
        launch_arguments=dict(use_sim_time=use_sim_time).items(),
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
                                            name='use_sim_time',
                                            default_value='true',
                                            description='Flag to enable usage of simulation time',
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui',
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=str(default_model_path),
            description='Absolute path to robot urdf file',
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=str(default_rviz_config_path),
            description='Absolute path to rviz config file',
        ),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
