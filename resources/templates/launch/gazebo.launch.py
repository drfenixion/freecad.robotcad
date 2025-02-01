import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    resources_package = '{package_name}'

    # Make path to resources dir without last package_name fragment.
    path_to_share_dir_clipped = ''.join(get_package_share_directory(resources_package).rsplit('/' + resources_package, 1))

    # Gazebo hint for resources.
    os.environ['GZ_SIM_RESOURCE_PATH'] = path_to_share_dir_clipped

    # Ensure `SDF_PATH` is populated since `sdformat_urdf` uses this rather

    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path


    use_custom_world = LaunchConfiguration('use_custom_world')
    use_custom_world_launch_arg = DeclareLaunchArgument('use_custom_world', default_value='true')
    gazebo_world = LaunchConfiguration('gazebo_world')
    gazebo_world_launch_arg = DeclareLaunchArgument('gazebo_world', default_value='empty.sdf')

    # prepare custom world
    world = os.getenv('GZ_SIM_WORLD', 'empty')
    fly_world_path = resources_package + '/worlds/' + world + '.sdf'
    gz_version = subprocess.getoutput("gz sim --versions")
    gz_version_major = re.search(r'^\d{{1}}', gz_version).group()
    launch_arguments=dict(gz_args = '-r ' + str(fly_world_path) + ' --verbose ', gz_version = gz_version_major).items()

    # Gazebo Sim.
    # by default the custom world is used, otherwise the gazebo world is used, which can be changed with the argument
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments=launch_arguments if use_custom_world else dict(gz_args='-r ' + gazebo_world + ' --verbose').items(),
    )

    # Spawn
    spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', '{robot_name}',
                '-x', '1.2',
                '-z', '2.3',
                '-Y', '3.4',
                '-topic', '/robot_description',
            ],
            output='screen',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value='true')

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(resources_package),
                    'launch',
                    'description.launch.py',
                ]),
            ]),
            condition=UnlessCondition(use_rviz),  # rviz launch includes rsp.
            launch_arguments=dict(use_sim_time=use_sim_time).items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(resources_package),
                'launch',
                'display.launch.py',
            ]),
        ]),
        condition=IfCondition(use_rviz),
        launch_arguments=dict(use_sim_time=use_sim_time).items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
        parameters=[{{
            'use_sim_time': use_sim_time,
        }}],
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        use_rviz_arg,
        use_custom_world_launch_arg,
        gazebo_world_launch_arg,
        robot_state_publisher,
        rviz,
        gazebo,
        spawn,
        gz_bridge,
    ])
