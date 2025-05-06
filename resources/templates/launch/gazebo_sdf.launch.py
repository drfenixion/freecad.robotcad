from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 

package='{package_name}'
gz_share_pkg=get_package_share_directory("ros_gz_sim")
share_dir=get_package_share_directory(package)
world_file=os.path.join(share_dir,"worlds","world.sdf")

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path=os.path.join(gz_share_pkg,"launch","gz_sim.launch.py")),
            launch_arguments=dict(gz_args="-r " +world_file).items()
            
        )
    ])