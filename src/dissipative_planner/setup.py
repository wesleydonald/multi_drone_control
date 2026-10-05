from glob import glob
from setuptools import find_packages, setup

package_name = 'dissipative_planner'

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
    description='Load planner with mid-flight detach/attach: OCP resize or the dissipative '
                'network (Quan et al.)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dissipative_planner = dissipative_planner.dissipative_node:main',
            'verify_dissipative = dissipative_planner.verify_dissipative:main',
        ],
    },
)
