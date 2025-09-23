from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def _load_text(path):
    with open(path, 'r') as f:
        return f.read()

def _xacro_to_urdf(xacro_path:str) -> str:
    try:
        import xacro
        doc = xacro.process_file(xacro_path)
        return doc.toprettyxml(indent='  ')
    except Exception:
        import subprocess, tempfile
        with tempfile.NamedTemporaryFile('r+', delete=False) as tmp:
            subprocess.run(['xacro', xacro_path, '-o', tmp.name], check=True)
            return open(tmp.name).read()

def generate_launch_description():
    pkg = get_package_share_directory('omx_pick_place_demo')
    URDF_XACRO = os.path.join(pkg, 'urdf',  'open_manipulator_x', 'open_manipulator_x.urdf.xacro')
    SRDF       = os.path.join(pkg, 'config','open_manipulator_x', 'open_manipulator_x.srdf')
    KIN        = os.path.join(pkg, 'config','open_manipulator_x', 'kinematics.yaml')
    CTRLS      = os.path.join(pkg, 'config','open_manipulator_x', 'moveit_controllers.yaml')
    JOINTLIM   = os.path.join(pkg, 'config','open_manipulator_x', 'joint_limits.yaml')

    assert os.path.exists(URDF_XACRO)
    assert os.path.exists(SRDF)
    assert os.path.exists(KIN)
    assert os.path.exists(CTRLS)
    assert os.path.exists(JOINTLIM)

    params = {}
    params["robot_description"]             = _xacro_to_urdf(URDF_XACRO)
    params["robot_description_semantic"]    = _load_text(SRDF)
    params["robot_description_kinematics"]  = _load_yaml(KIN)
    params["robot_description_planning"]    = _load_yaml(JOINTLIM)
    params["planning_pipelines"]            = ["ompl"]
    params.update(_load_yaml(CTRLS) or {})

    node = Node(
        package="omx_pick_place_demo",
        executable="moveit_pick_place",
        output="screen",
        parameters=[params],
    )
    return LaunchDescription([node])
