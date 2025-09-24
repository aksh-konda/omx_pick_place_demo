from setuptools import setup

package_name = 'omx_pick_place_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py',
                                               'launch/spawn_cube.launch.py',
                                               'launch/run_pick_place.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/empty.world']),
        ('share/' + package_name + '/models/cube_10cm', ['models/cube_10cm/model.sdf',
                                                         'models/cube_10cm/model.config']),
        ('share/' + package_name + '/models/cube_5cm', ['models/cube_5cm/model.sdf','models/cube_5cm/model.config']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='OMX pick-and-place demo (Gazebo + MoveItPy fallback to Gazebo API).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_pick_place = omx_pick_place_demo.moveit_pick_place:main',
            'gazebo_move_cube = omx_pick_place_demo.gazebo_move_cube:main',
            'spawn_cube_gz = omx_pick_place_demo.spawn_cube_gz:main',
        ],
    },
)
