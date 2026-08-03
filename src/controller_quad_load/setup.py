from glob import glob
from setuptools import find_packages, setup

package_name = 'controller_quad_load'

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
    maintainer='mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='Cable-aware MPC drone control (models cable tension force)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main = controller_quad_load.main:main',
            'controller = controller_quad_load.controller_mpc:main',
            'kt_estimator = controller_quad_load.thrust_ratio_node:main',
        ],
    },
)
