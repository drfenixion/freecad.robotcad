from pathlib import Path

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = Path(FindPackageShare(package='{package_name}').find('{package_name}'))
    default_model_path = pkg_share / 'urdf/{xacro_wrapper_file}'
    default_rviz_config_path = pkg_share / 'rviz/robot_description.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('{package_name}'),
                'launch',
                'description.launch.py',
            ]),
        ]),
        launch_arguments=dict(
            use_sim_time=use_sim_time,
            use_ros2_control=use_ros2_control
        ).items(),
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{{
            'use_sim_time': use_sim_time,
        }}],
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{{
            'use_sim_time': use_sim_time,
        }}],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{{
            'use_sim_time': use_sim_time,
        }}],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Flag to enable usage of simulation time',
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui',
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=str(default_model_path),
            description='Absolute path to robot urdf file',
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=str(default_rviz_config_path),
            description='Absolute path to rviz config file',
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Include ros2_control configuration in URDF'
        ),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
