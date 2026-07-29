from setuptools import setup

package_name = 'vb300_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/waypoints.yaml',
            'config/vehicle.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/vb300.launch.py',
            'launch/livox_view.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ins_node    = vb300_nodes.ins_node:main',
            'ins_nav_node = vb300_nodes.ins_nav_node:main',
            'cloud_relay_node = vb300_nodes.cloud_relay_node:main',
            'waypoint_recorder = vb300_nodes.waypoint_recorder:main',
            'grid_node   = vb300_nodes.grid_node:main',
            'planner_node = vb300_nodes.planner_node:main',
            'control_node = vb300_nodes.control_node:main',
            'can_node    = vb300_nodes.can_node:main',
            'keyboard_teleop = vb300_nodes.keyboard_teleop:main',
            'send_cmd   = vb300_nodes.send_cmd:main',
            'livox_udp = vb300_nodes.livox_udp_bridge:main',
        ],
    },
)
