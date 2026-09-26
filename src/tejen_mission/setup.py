from setuptools import setup
from glob import glob
import os

package_name = 'tejen_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'config', 'payloads'),
            glob('config/payloads/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for quadcopter communication.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'irl_hover_reference = tejen_mission.irl_hover_reference:main',
            'elrs_interface_irl = tejen_mission.elrs_interface_irl:main',
            'magnet_attachment_manager_irl = tejen_mission.magnet_attachment_manager_irl:main',
            'motion_capture_publisher_irl = tejen_mission.motion_capture_publisher_irl:main',
            'elrs_interface = tejen_mission.elrs_interface:main',
            'video_interface = tejen_mission.video_interface:main',
            'motion_capture_publisher_node = tejen_mission.motion_capture_publisher_node:main',
            'fake_cooperative_transport_world = tejen_mission.fake_cooperative_transport_world:main',
            'ring_bridge = tejen_mission.ring_bridge:main',
            'fake_moving_obstacle = tejen_mission.fake_moving_obstacle:main',
            'online_join_planner = tejen_mission.online_join_planner:main',
            'pendulum_state_publisher = tejen_mission.pendulum_state_publisher:main',
            'magnet_tip_publisher = tejen_mission.magnet_tip_publisher:main',
            'magnet_attachment_manager = tejen_mission.magnet_attachment_manager:main',
            'm2a_physical_capture_manager = tejen_mission.m2a_physical_capture_manager:main',
            'm2_attachment_observer = tejen_mission.m2_attachment_observer:main',
            'm2b_b1_telemetry = tejen_mission.m2b_b1_telemetry:main',
            'm2a_gazebo_joint_truth_bridge = tejen_mission.m2a_gazebo_joint_truth_bridge:main',
            'm2b_ground_initializer = tejen_mission.m2b_ground_initializer:main',
            'm2c_fleet_manager = tejen_mission.m2_fleet_manager:main',
            'm2_irl_mocap_router = tejen_mission.m2_irl_mocap_router:main',
            'm2_irl_fleet_manager = tejen_mission.m2_irl_fleet_manager:main',
            'm2d_fleet_supervisor = tejen_mission.m2d_fleet_supervisor:main',
            'm2d_ring_commitment = tejen_mission.m2d_ring_commitment:main',
            'm2c_ground_spawn = tejen_mission.m2c_ground_spawn:main',
            'm2b_b0_telemetry = tejen_mission.m2b_b0_telemetry:main',
            'pickup_mission_planner = tejen_mission.pickup_mission_planner:main',
            'simulation_test_supervisor = tejen_mission.simulation_test_supervisor:main',
            'simulation_xy_disturbance_gate = tejen_mission.simulation_xy_disturbance_gate:main',
        ],
    },
)
