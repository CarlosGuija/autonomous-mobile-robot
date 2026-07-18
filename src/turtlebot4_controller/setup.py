from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlebot4_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos',
    maintainer_email='carlos@users.noreply.github.com',
    description='Odometry-based motion controllers and behaviors for TurtleBot 4.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'velocity_publisher = turtlebot4_controller.velocity_publisher:main',
            'drive_distance = turtlebot4_controller.drive_distance:main',
            'rotate_angle = turtlebot4_controller.rotate_angle:main',
            'drive_distance_action_server = '
            'turtlebot4_controller.drive_distance_action_server:main',
            'rotate_angle_action_server = '
            'turtlebot4_controller.rotate_angle_action_server:main',
            'move_square = turtlebot4_controller.move_square:main',
        ],
    },
)
