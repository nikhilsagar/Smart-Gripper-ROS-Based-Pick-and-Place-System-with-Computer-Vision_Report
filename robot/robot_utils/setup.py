from setuptools import find_packages, setup

package_name = 'robot_utils'

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
    maintainer='nikhil',
    maintainer_email='nikhil@todo.todo',
    description='Utility package for robot position verification and motion control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_position_command = robot_utils.send_position_command:main',
            'robotic_arm_pick_place_node = robot_utils.robotic_arm_pick_place_node:main',
        ],
    },
)
