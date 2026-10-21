from glob import glob
from setuptools import find_packages, setup

name = 'oak_depth_navigation'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md', 'TEST_RESULTS.md']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/config', glob('config/*')),
        ('share/' + name + '/rviz', glob('rviz/*')),
        ('share/' + name + '/urdf', glob('urdf/*')),
        ('share/' + name + '/firmware', glob('firmware/*')),
        ('share/' + name + '/dashboard', glob('dashboard/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Arnab', maintainer_email='arnab@example.com',
    description='OAK-D Pro depth/IMU mapping and navigation', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'check_mapping = oak_depth_navigation.check_mapping:main',
        'save_map = oak_depth_navigation.save_map:main',
        'serial_hub = oak_depth_navigation.serial_hub_node:main',
        'diff_drive = oak_depth_navigation.diff_drive_node:main',
        'wheel_odom = oak_depth_navigation.wheel_odom_node:main',
        'command_mux = oak_depth_navigation.command_mux:main',
        'dashboard = oak_depth_navigation.dashboard_node:main',
    ]},
)
