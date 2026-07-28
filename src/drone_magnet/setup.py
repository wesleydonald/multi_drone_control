from setuptools import find_packages, setup

package_name = 'drone_magnet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wesley Donald',
    maintainer_email='wesleydonaldnz2@gmail.com',
    description='Swung-electromagnet ATTACH support nodes + ELRSCommand handoff mux.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'online_join_planner = drone_magnet.online_join_planner:main',
            'attach_target_publisher = drone_magnet.attach_target_publisher:main',
            'magnet_attachment_manager = drone_magnet.magnet_attachment_manager:main',
            'magnet_tip_publisher = drone_magnet.magnet_tip_publisher:main',
            'pendulum_state_publisher = drone_magnet.pendulum_state_publisher:main',
            'elrs_mux = drone_magnet.elrs_mux:main',
        ],
    },
)
