from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller_graffiti'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # This line tells colcon to install your data and font files
    ('share/' + package_name, ['controller_graffiti/layout_data.txt', 'controller_graffiti/Inlanders Demo.otf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main = controller_graffiti.main:main',
            'input_node = controller_graffiti.input_node:main',
        ],
    },
)
