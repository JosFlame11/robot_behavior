from setuptools import find_packages, setup

package_name = 'robot_behavior'

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
    maintainer='josflame11',
    maintainer_email='jolapa20@gmail.com',
    description='This packages controlls the robot decision making. Sending commands to the robot to perform actions.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensors_node = robot_behavior.sensor_avoidance:main',
            'path_tracking_node = robot_behavior.path_tracking:main'
        ],
    },
)
