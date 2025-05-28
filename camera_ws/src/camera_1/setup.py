from setuptools import find_packages, setup
import os

package_name = 'camera_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikhil',
    maintainer_email='nikhil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # ✅ keeps ROS aware of your package
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), ['resource/squares_config.json'])  # ✅ adds JSON
    ],
    entry_points={
        'console_scripts': [
            'camera_1 = camera_1.first_camera:main',
            'generate_aruco = camera_1.generate_aruco:main',
            'detect_aruco = camera_1.detect_aruco:main',
            'check_1 = camera_1.check_1:main',
            'check_2 = camera_1.check_2:main',
            'check_3 = camera_1.check_3:main',
            'check_4 = camera_1.check_4:main',
            'map_objects = camera_1.map_objects:main'

        ],
    },
)
