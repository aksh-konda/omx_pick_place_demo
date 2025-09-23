
import sys, subprocess, re

def detect_world():
    try:
        out = subprocess.check_output(['gz','service','-l'], text=True)
        m = re.search(r'^/world/([^/]+)/set_pose$', out, flags=re.M)
        return m.group(1) if m else 'empty'
    except Exception:
        return 'empty'

def gz_service(service, msg_type, payload):
    for flag in ('-r','-m'):
        cmd = ['gz','service','-s', service, flag, msg_type, '-p', payload]
        res = subprocess.run(cmd, check=False)
        if res.returncode == 0:
            return 0
    return res.returncode

def main():
    x = float(sys.argv[1]) if len(sys.argv)>1 else 0.35
    y = float(sys.argv[2]) if len(sys.argv)>2 else 0.0
    z = float(sys.argv[3]) if len(sys.argv)>3 else 0.05
    world = detect_world()
    body = f"name: 'cube_5cm', position: {{ x: {x}, y: {y}, z: {z} }}"
    rc = gz_service(f"/world/{world}/set_pose", "gz.msgs.Pose", body)
    if rc != 0:
        print("[gazebo_move_cube] set_pose failed (rc=", rc, ")", sep="")

if __name__ == '__main__':
    main()
