from setuptools import setup
import os
from glob import glob

package_name = 'yolo_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the bus.jpg file
        (os.path.join('share', package_name), glob('bus.jpg'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='A ROS2 package for YOLO object detection.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = yolo_ros2.publisher:main',
            'subscriber = yolo_ros2.subscriber:main',
        ],
    },
)
