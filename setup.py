from glob import glob
from setuptools import setup

package_name = 'pistar_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Harsh',
    description='ROS 2 base stack for the Pi Star Lab RC car',
    license='MIT',
    entry_points={
        'console_scripts': [
            'actuator_node = pistar_car.actuator_node:main',
            'teleop_keyboard = pistar_car.teleop_keyboard:main',
            'teleop_joy = pistar_car.teleop_joy:main',
            'lane_follower = pistar_car.lane_follower:main',
            'csi_camera = pistar_car.csi_camera:main',
        ],
    },
)
