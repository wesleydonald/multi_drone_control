from setuptools import find_packages, setup

package_name = 'controller_wind_rl'

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
    maintainer='mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='Basic RL drone control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main = controller_wind_rl.main:main'
        ],
    },
)
