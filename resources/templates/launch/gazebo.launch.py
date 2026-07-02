import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression

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


    gazebo_world = LaunchConfiguration('gazebo_world')
    gazebo_world_launch_arg = DeclareLaunchArgument('gazebo_world', default_value='empty.sdf')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Include ros2_control configuration in URDF'
    )

    # prepare custom world
    world_env = os.getenv('GZ_SIM_WORLD')
    if world_env:
        fly_world_path = resources_package + '/worlds/' + world_env + '.sdf'
        gz_version = subprocess.getoutput("gz sim --versions")
        gz_version_major = re.search(r'^\d{{1}}', gz_version).group()
        launch_arguments=dict(gz_args = '-r ' + str(fly_world_path) + ' --verbose ', gz_version = gz_version_major).items()

    # Gazebo Sim.
    # by default the custom world is used, otherwise the gazebo world is used, which can be changed with the argument
    # If GZ_SIM_WORLD env var is set, it takes precedence
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments=launch_arguments if world_env else dict(gz_args='-r ' + gazebo_world + ' --verbose').items(),
    )

    # Spawn
    spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', '{robot_name}',
                '-x', '1.2',
                '-z', '1.0',
                '-Y', '3.4',
                '-topic', '/robot_description',
            ],
            output='screen',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value='true')
    use_px4 = LaunchConfiguration('use_px4')
    use_px4_arg = DeclareLaunchArgument('use_px4', default_value='false')
    use_sverk = LaunchConfiguration('use_sverk')
    use_sverk_arg = DeclareLaunchArgument('use_sverk', default_value='false')

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(resources_package),
                    'launch',
                    'description.launch.py',
                ]),
            ]),
            condition=UnlessCondition(use_rviz),  # rviz launch includes rsp.
            launch_arguments=dict(
                use_sim_time=use_sim_time,
                use_ros2_control=use_ros2_control
            ).items(),
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
        launch_arguments=dict(
            use_sim_time=use_sim_time,
            use_ros2_control=use_ros2_control
        ).items(),
    )

    gz_bridge_parameter = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', {sensors_bridge_args}],
        output='screen',
        parameters=[{{
            'use_sim_time': use_sim_time,
        }}],
    )

    gz_bridge_camera = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[{cameras_topics_comma_sep}],
        output='screen'
    )
    gz_bridge_camera_dummy = DeclareLaunchArgument('', default_value='') # dummy for LaunchDescription could take empty element

    # Sverk offboard_control
    offboard_control = Node(
        package='offboard_control',
        executable='offboard_control',
        name='offboard_control',
        condition=IfCondition(use_sverk),
        parameters=[
            {{'simulator': True}}
        ],
        output='screen'
    )

    px4_condition = PythonExpression(['"', use_px4, '" == "true" or "', use_sverk, '" == "true"'])
    px4 = ExecuteProcess(
        cmd=['px4'],
        name='px4',
        output='screen',
        shell=True,
        condition=IfCondition(px4_condition),
        env={{
            **os.environ,
            'PX4_GZ_STANDALONE': '1',
            'PX4_SYS_AUTOSTART': '{px4AirframeId}',
            'PX4_GZ_MODEL_NAME': '{robot_name}',
        }}
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        use_rviz_arg,
        use_px4_arg,
        use_sverk_arg,
        gazebo_world_launch_arg,
        use_ros2_control_arg,
        robot_state_publisher,
        rviz,
        gazebo,
        spawn,
        gz_bridge_parameter,
        (gz_bridge_camera if [{cameras_topics_comma_sep}] else gz_bridge_camera_dummy),
        offboard_control,
        px4,
    ])
