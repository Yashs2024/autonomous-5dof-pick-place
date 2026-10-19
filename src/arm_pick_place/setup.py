import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_pick_place'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash_surve',
    maintainer_email='yash_surve@todo.todo',
    description='Autonomous 5-DOF Vision-Guided Pick and Place with Dynamic IK & Hardware Bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_node = arm_pick_place.pick_place_node:main',
            'vision_pick_place_node = arm_pick_place.vision_pick_place_node:main',
            'yolov8_pick_place_node = arm_pick_place.yolov8_pick_place_node:main',
            'servo_serial_bridge = arm_pick_place.servo_serial_bridge:main',
        ],
    },
)
