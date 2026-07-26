from glob import glob
from setuptools import find_packages, setup

package_name = 'controller_dissipative'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wesley Donald',
    maintainer_email='wesleydonaldnz2@gmail.com',
    description='Decentralized dissipative detach controller for a cable-suspended '
                'load (Quan et al., table-mechanics-inspired self-organizing swarm)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dissipative = controller_dissipative.dissipative_node:main',
            'verify_dissipative = controller_dissipative.verify_dissipative:main',
        ],
    },
)
