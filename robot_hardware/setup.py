import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # include all files in config
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bouchier',
    maintainer_email='paul.bouchier@gmail.com',
    description='Package contains nodes that talk to robot hardware (including ESP32)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp_link = robot_hardware.esp_link:main',
            'ping_esp = robot_hardware.ping_esp:main',
            'reset_esp = robot_hardware.reset_esp:main',
            'bit_mode = robot_hardware.bit_mode:main',
            'clear_odom = robot_hardware.clear_odom:main',
            'dgnss_mon = robot_hardware.dgnss_mon:main'
        ],
    },
)
