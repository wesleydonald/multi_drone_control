from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller_cable_payload'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wesley Donald',
    maintainer_email='wesleydonaldnz2@gmail.com',
    description='TU-Delft-inspired cable-suspended payload controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner = controller_cable_payload.planner:main',
            'drone_controller = controller_cable_payload.drone_controller:main',
            'payload_mocap_emulator = controller_cable_payload.payload_mocap_emulator:main',
            'payload_betaflight_comm = controller_cable_payload.payload_betaflight_comm:main',
        ],
    },
)
