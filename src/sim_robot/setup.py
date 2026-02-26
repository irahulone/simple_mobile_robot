from setuptools import find_packages, setup

package_name = 'sim_robot'

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
    maintainer='neo',
    maintainer_email='neo.yuichiro@megachips.co.jp',
    description='Simulated differential-drive robot that integrates wheel velocities into 2-D pose.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_robot_node = sim_robot.sim_robot_node:main',
        ],
    },
)
