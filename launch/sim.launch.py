from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ROBOTIS OM-X gazebo bringup (assumes open_manipulator is built in this workspace)
    omx_pkg = get_package_share_directory('open_manipulator_bringup')
    omx_gz_launch = os.path.join(omx_pkg, 'launch', 'open_manipulator_x_gazebo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(omx_gz_launch)),
    ])
