from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Xacro args
    prefix_arg   = DeclareLaunchArgument('prefix', default_value='')
    use_fake_arg = DeclareLaunchArgument('use_fake_hardware', default_value='true')
    use_sim_arg  = DeclareLaunchArgument('use_sim', default_value='true')

    prefix   = LaunchConfiguration('prefix')
    use_fake = LaunchConfiguration('use_fake_hardware')
    use_sim  = LaunchConfiguration('use_sim')

    # 1) URDF/Xacro from open_manipulator_description
    om_desc_share = get_package_share_directory('open_manipulator_description')
    urdf_xacro = str(Path(om_desc_share) / 'urdf' / 'open_manipulator_x' / 'open_manipulator_x.urdf.xacro')

    # 2) All MoveIt configs from open_manipulator_moveit_config
    om_moveit_share = get_package_share_directory('open_manipulator_moveit_config')
    cfg = Path(om_moveit_share) / 'config' / 'open_manipulator_x'

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='open_manipulator_x',
            package_name='open_manipulator_moveit_config'
        )
        .robot_description(
            urdf_xacro,
            mappings={'prefix': prefix, 'use_fake_hardware': use_fake, 'use_sim': use_sim}
        )
        .robot_description_semantic(str(cfg / 'open_manipulator_x.srdf'))
        .joint_limits(str(cfg / 'joint_limits.yaml'))
        .trajectory_execution(str(cfg / 'moveit_controllers.yaml'))
        .robot_description_kinematics(str(cfg / 'kinematics.yaml'))
        .to_moveit_configs()
    )

    return LaunchDescription([
        prefix_arg, use_fake_arg, use_sim_arg,
        Node(
            package='omx_pick_place_demo',
            executable='moveit_pick_place',
            output='screen',
            parameters=[moveit_config.to_dict()],
        ),
    ])
