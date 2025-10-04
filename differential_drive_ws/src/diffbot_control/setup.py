from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diffbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovasura',
    maintainer_email='vanshmalani27@gmail.com',
    description='Motor control and hardware interface for differential drive robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_driver = diffbot_control.motor_driver:main',
            'motor_tester = diffbot_control.motor_tester:main',
            'teleop_keyboard = diffbot_control.teleop_keyboard:main',
            'robot_controller = diffbot_control.robot_controller:main',
            'robot_control_client = diffbot_control.robot_control_client:main',
        ],
    },
)