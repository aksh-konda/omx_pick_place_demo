import subprocess, re, sys, os
from ament_index_python.packages import get_package_share_directory

def detect_world():
    out = subprocess.check_output(['gz','service','-l'], text=True)
    m  = re.search(r'^/world/([^/]+)/create$', out, flags=re.M)
    return m.group(1) if m else 'empty'

def gz_call(service, reqtype, req, reptype='gz.msgs.Boolean', timeout='2000'):
    return subprocess.run([
        'gz','service','-s', service,
        '--reqtype', reqtype,
        '--reptype', reptype,
        '--timeout', timeout,
        '--req', req
    ], check=False).returncode

def main():
    world = detect_world()
    pkg   = get_package_share_directory('omx_pick_place_demo')
    sdf   = os.path.join(pkg, 'models', 'cube_5cm', 'model.sdf')
    print(f"[spawn_cube] world={world}")
    # best-effort remove (ignore failures)
    gz_call(f'/world/{world}/remove', 'gz.msgs.Entity', "name: 'cube_5cm'")
    # spawn 5 cm cube
    req = f"sdf_filename: '{sdf}', name: 'cube_5cm', pose: {{ position: {{ x: 0.18, y: 0.0, z: 0.05 }} }}"
    rc  = gz_call(f'/world/{world}/create', 'gz.msgs.EntityFactory', req)
    if rc != 0:
        print(f"[spawn_cube] FAILED rc={rc}")
        sys.exit(rc)
    print("[spawn_cube] cube_5cm spawned.")
    sys.exit(0)

if __name__ == '__main__':
    main()
