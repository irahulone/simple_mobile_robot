from setuptools import setup
import os
from glob import glob

package_name = 'virtual_joypad'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='Virtual teleop controllers with PyQt6 GUI for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_controller = virtual_joypad.virtual_controller:main',
            'virtual_joy = virtual_joypad.virtual_joy:main',
            'virtual_joy_slider = virtual_joypad.virtual_joy_slider:main',
        ],
    },
)
