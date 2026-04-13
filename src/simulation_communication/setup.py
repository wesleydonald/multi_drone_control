from setuptools import find_packages, setup

package_name = 'simulation_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', ['launch/betaflight_linear_simulation_launch.py', 'launch/betaflight_simulation_launch.py', 'launch/betaflight_angle_simulation_launch.py', 'launch/omnicopter_simulation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ELRS_pass_through = simulation_communication.ELRS_pass_through:main',
            'motion_capture_emulator = simulation_communication.motion_capture_emulator:main',
            'betaflight_communication = simulation_communication.betaflight_communication:main',
            'angle_betaflight_communication = simulation_communication.angle_betaflight_communication:main',
            'pendulum_state_listener = simulation_communication.pendulum_state_listener:main',
            'orbslam_emulator = simulation_communication.orbslam_emulator:main',
        ],
    },
)
