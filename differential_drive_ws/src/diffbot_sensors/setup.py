from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diffbot_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovasura',
    maintainer_email='vanshmalani27@gmail.com',
    description='Front ultrasonic sensor node for obstacle avoidance',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'front_ultrasonic_node.py = diffbot_sensors.front_ultrasonic_node:main',
        ],
    },
)
