import sys
import subprocess
import re
import rclpy
from rclpy.node import Node

# Try MoveItPy; if not available or fails at runtime, we'll fallback to gz.
try:
    from moveit.planning import MoveItPy
    HAVE_MOVEITPY = True
except Exception:
    HAVE_MOVEITPY = False

from geometry_msgs.msg import PoseStamped

CUBE_NAME = 'cube_5cm'
BASE_FRAME = 'base_link'           # OM-X base frame in the ROBOTIS config
EEF_LINK   = 'end_effector_link'   # Common OM-X end-effector link

def gz_call(service: str, reqtype: str, req: str, reptype: str = 'gz.msgs.Boolean', timeout: str = '2000') -> int:
    """Call Gazebo service using long flags (--reqtype/--req). Returns 0 on success."""
    cmd = [
        'gz','service','-s', service,
        '--reqtype', reqtype,
        '--reptype', reptype,
        '--timeout', timeout,
        '--req', req
    ]
    return subprocess.run(cmd, check=False).returncode

def detect_world() -> str:
    """Detect active world name from gz."""
    try:
        out = subprocess.check_output(['gz','service','-l'], text=True)
        m = re.search(r'^/world/([^/]+)/set_pose$', out, flags=re.M)
        return m.group(1) if m else 'empty'
    except Exception:
        return 'empty'

def fallback_move_cube(x: float = 0.38, y: float = 0.0, z: float = 0.05):
    """Move the cube using Gazebo set_pose (no MoveIt required)."""
    world = detect_world()
    req   = f"name: '{CUBE_NAME}', position: {{ x: {x}, y: {y}, z: {z} }}"
    rc    = gz_call(f'/world/{world}/set_pose', 'gz.msgs.Pose', req)
    print(f'[fallback] gz set_pose rc={rc}')

def make_pose(x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = BASE_FRAME
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.w = qw
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    return p

def run_moveit_sequence():
    """Attempt a minimal pick->place with MoveIt; raise on failure."""
    moveit = MoveItPy(node_name='omx_moveit_py')
    robot  = moveit.get_robot_model()

    # Planning components (OM-X SRDF commonly names these 'arm' and 'gripper')
    arm = moveit.get_planning_component('arm')
    try:
        gripper = moveit.get_planning_component('gripper')
    except Exception:
        gripper = None

    # 5 cm cube sits with center at z≈0.05. Approach low but safe.
    pre   = make_pose(0.18, 0.00, 0.16)
    over  = make_pose(0.18, 0.00, 0.09)
    lift  = make_pose(0.18, 0.00, 0.18)
    place = make_pose(0.38, 0.00, 0.12)

    def plan_exec(target_pose: PoseStamped):
        arm.set_goal_state(pose_stamped_msg=target_pose, pose_link=EEF_LINK)
        plan = arm.plan()
        if not plan:
            raise RuntimeError('Planning failed')
        ok = arm.execute()
        if not ok:
            raise RuntimeError('Execution failed')

    # Open gripper (if available)
    if gripper:
        try:
            # Typical OM-X gripper joint name is 'gripper'; ranges vary across configs.
            gripper.set_goal_state(configuration={'gripper': 0.01})
            if gripper.plan():
                gripper.execute()
        except Exception:
            pass

    # Approach
    for target in (pre, over):
        plan_exec(target)

    # Close gripper (if available)
    if gripper:
        try:
            gripper.set_goal_state(configuration={'gripper': -0.01})
            if gripper.plan():
                gripper.execute()
        except Exception:
            pass

    # Lift and place
    for target in (lift, place):
        plan_exec(target)

    # Open to release (if available)
    if gripper:
        try:
            gripper.set_goal_state(configuration={'gripper': 0.01})
            if gripper.plan():
                gripper.execute()
        except Exception:
            pass

    print('[moveit] Pick-&-place sequence attempted (5cm cube).')

def main():
    # Keep this synchronous to avoid timer/indentation issues.
    # Try MoveIt, fallback to gz on any error.
    try:
        if not HAVE_MOVEITPY:
            raise RuntimeError('MoveItPy import unavailable')
        rclpy.init()
        try:
            run_moveit_sequence()
        finally:
            rclpy.shutdown()
    except Exception as e:
        print('[moveit] Error or unavailable:', e)
        fallback_move_cube()

if __name__ == '__main__':
    main()
